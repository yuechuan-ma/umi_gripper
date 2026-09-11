"""支持一至两个独立通道的总线舵机夹爪控制。"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from serial.tools import list_ports
from scservo_sdk import (
    COMM_SUCCESS,
    PortHandler,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_LOAD_L,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_TORQUE_ENABLE,
    sms_sts,
)

STEPS_PER_TURN, MAX_MULTI_TURN_STEPS = 4096, 28672
DEFAULT_BAUDRATES = (1_000_000, 500_000, 250_000, 115_200, 57_600, 38_400)
DEFAULT_CONFIG_PATH = Path(__file__).with_name("gripper_config.json")


class GripperError(RuntimeError):
    pass


class ConfigError(GripperError):
    pass


def _read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"找不到配置文件：{path}") from exc
    except json.JSONDecodeError as exc:
        raise ConfigError(f"配置文件不是有效的 JSON：{exc}") from exc


def save_config(config: dict, path: str | Path = DEFAULT_CONFIG_PATH) -> None:
    Path(path).write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def default_servo_config(selected: dict | None = None) -> dict:
    selected = selected or {}
    return {
        "serial_port": selected.get("port"),
        "servo_id": selected.get("servo_id"),
        "baudrate": selected.get("baudrate"),
        "servo_model_number": selected.get("model"),
        "closed_position_steps": None,
        "neutral_position_steps": None,
        "open_position_steps": None,
        "speed": 1200,
        "grip_strength": 50,
        "control_frequency_hz": 20,
        "status_frequency_hz": 50,
        "calibration_step_steps": 128,
        "motion_timeout_s": 30,
        "contact_load_threshold": None,
        "contact_current_ma": None,
    }


def _integer(config: dict, key: str, low: int, high: int) -> int:
    value = config.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not low <= value <= high
    ):
        raise ConfigError(f"配置项“{key}”必须是 {low} 到 {high} 的整数。")
    return value


def _servos(config: dict) -> list[dict | None]:
    if "servos" not in config:
        return [dict(config), None]
    value = config["servos"]
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(x is not None and not isinstance(x, dict) for x in value)
    ):
        raise ConfigError("配置项“servos”必须是包含两个配置对象或 null 的列表。")
    return [dict(x) if x is not None else None for x in value]


def _validate_servo(config: dict, channel: int, calibrated: bool) -> None:
    required = (
        "serial_port",
        "servo_id",
        "baudrate",
        "speed",
        "grip_strength",
        "control_frequency_hz",
        "status_frequency_hz",
        "calibration_step_steps",
        "motion_timeout_s",
    )
    missing = [key for key in required if config.get(key) in (None, "")]
    if missing:
        raise ConfigError(f"{channel} 号舵机配置尚未完成：" + "、".join(missing))
    if not isinstance(config["serial_port"], str):
        raise ConfigError(f"{channel} 号舵机串口无效。")
    _integer(config, "servo_id", 0, 253)
    _integer(config, "baudrate", 9600, 1_000_000)
    _integer(config, "speed", 1, 3400)
    _integer(config, "grip_strength", 1, 100)
    control = _integer(config, "control_frequency_hz", 1, 100)
    if _integer(config, "status_frequency_hz", 1, 100) < control:
        raise ConfigError(f"{channel} 号舵机状态频率不能低于控制频率。")
    _integer(config, "calibration_step_steps", 10, 1024)
    _integer(config, "motion_timeout_s", 2, 120)
    load, current = config.get("contact_load_threshold"), config.get(
        "contact_current_ma"
    )
    if (load is None) != (current is None):
        raise ConfigError("夹紧阈值必须同时填写，或同时留空。")
    if load is not None:
        _integer(config, "contact_load_threshold", 1, 1000)
        _integer(config, "contact_current_ma", 1, 5000)
    if not calibrated:
        return
    for key in (
        "closed_position_steps",
        "neutral_position_steps",
        "open_position_steps",
    ):
        _integer(config, key, -MAX_MULTI_TURN_STEPS, MAX_MULTI_TURN_STEPS)
    if config["closed_position_steps"] != 0:
        raise ConfigError(f"{channel} 号舵机闭合参考位置必须为 0。")
    opened, neutral = config["open_position_steps"], config["neutral_position_steps"]
    if abs(opened) < 10 or opened * neutral < 0 or abs(neutral) > abs(opened):
        raise ConfigError(f"{channel} 号舵机标定位置不合理。")


def validate_config(config: dict, *, calibrated: bool = True) -> None:
    servos = _servos(config)
    active = [(i, x) for i, x in enumerate(servos) if x is not None]
    if not active:
        raise ConfigError("至少需要配置一个舵机。")
    seen, ports = set(), {}
    for index, item in active:
        _validate_servo(item, index, calibrated)
        identity = (item["serial_port"], item["servo_id"])
        if identity in seen:
            raise ConfigError("同一串口上的两个舵机 ID 不能相同。")
        seen.add(identity)
        if (
            item["serial_port"] in ports
            and ports[item["serial_port"]] != item["baudrate"]
        ):
            raise ConfigError("同一串口上的两个舵机必须使用相同波特率。")
        ports[item["serial_port"]] = item["baudrate"]


def _raw_delta(previous: int, current: int) -> int:
    delta = (current - previous) % STEPS_PER_TURN
    return delta - STEPS_PER_TURN if delta > STEPS_PER_TURN // 2 else delta


class ServoBus:
    """一个物理串口，可供多个不同 ID 的舵机使用。"""

    def __init__(self, port: str, baudrate: int):
        self.port_name, self.baudrate = port, baudrate
        self.port, self.packet, self.opened = PortHandler(port), None, False

    @staticmethod
    def _check(result: int, error: int, action: str) -> None:
        if result != COMM_SUCCESS:
            raise GripperError(f"{action}失败：串口通信异常（代码 {result}）。")
        if error:
            raise GripperError(f"{action}失败：舵机报告异常（代码 {error}）。")

    def open(self) -> None:
        try:
            self.opened = self.port.openPort()
        except Exception as exc:
            raise GripperError(f"无法打开串口 {self.port_name}：{exc}") from exc
        if not self.opened:
            raise GripperError(f"无法打开串口 {self.port_name}。")
        if not self.port.setBaudRate(self.baudrate):
            self.close()
            raise GripperError(f"无法设置串口速度 {self.baudrate}。")
        self.packet = sms_sts(self.port)

    def close(self) -> None:
        if self.opened:
            self.port.closePort()
            self.opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def _read1(self, sid, address, action):
        value, result, error = self.packet.read1ByteTxRx(sid, address)
        self._check(result, error, action)
        return value

    def _read2(self, sid, address, action):
        value, result, error = self.packet.read2ByteTxRx(sid, address)
        self._check(result, error, action)
        return value

    def _write1(self, sid, address, value, action):
        result, error = self.packet.write1ByteTxRx(sid, address, value)
        self._check(result, error, action)

    def _write2(self, sid, address, value, action):
        result, error = self.packet.write2ByteTxRx(sid, address, value)
        self._check(result, error, action)

    def verify(self, sid: int) -> int:
        model, result, error = self.packet.ping(sid)
        self._check(result, error, f"确认 ID {sid} 舵机")
        return model

    def configure_speed_mode(self, sid: int) -> None:
        self._write1(sid, SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")
        self._write1(sid, 55, 0, "解锁舵机设置")
        self._write2(sid, 9, 0, "设置最小角度")
        self._write2(sid, 11, 0, "设置最大角度")
        self._write1(sid, 33, 1, "切换多圈速度模式")
        self._write1(sid, 55, 1, "锁定舵机设置")

    def check_speed_mode(self, sid: int) -> None:
        if self._read1(sid, 33, "读取舵机模式") != 1:
            raise GripperError(
                f"ID {sid} 舵机未处于多圈速度模式，请先运行 gripper_init.py。"
            )

    def prepare(self, sid: int, strength: int) -> None:
        self._write2(sid, 48, strength * 10, "设置夹紧力度")
        self._write1(sid, SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")

    def change_id(self, old_id: int, new_id: int) -> None:
        self._write1(old_id, 55, 0, "解锁舵机设置")
        self._write1(old_id, 5, new_id, "写入新 ID")
        self._write1(new_id, 55, 1, "锁定舵机设置")
        self.verify(new_id)

    def set_speed(self, sid: int, speed: int) -> None:
        result, error = self.packet.WriteSpec(sid, speed, 50)
        self._check(result, error, "下发转速")

    def telemetry(self, sid: int) -> dict:
        position, result, error = self.packet.ReadPos(sid)
        self._check(result, error, "读取当前位置")
        load = self._read2(sid, SMS_STS_PRESENT_LOAD_L, "读取负载")
        current = self._read2(sid, SMS_STS_PRESENT_CURRENT_L, "读取电流")
        temp = self._read1(sid, SMS_STS_PRESENT_TEMPERATURE, "读取温度")
        load = -(load & ~(1 << 10)) if load & (1 << 10) else load
        current = -(current & ~(1 << 15)) if current & (1 << 15) else current
        return {
            "raw_position": position % STEPS_PER_TURN,
            "load": load,
            "current_ma": round(current * 6.5),
            "temperature_c": temp,
        }


def _scan(baudrates: tuple[int, ...]) -> list[dict]:
    found = []
    for info in list_ports.comports():
        handler, opened = PortHandler(info.device), False
        try:
            try:
                opened = handler.openPort()
            except Exception:
                continue
            if not opened:
                continue
            packet = sms_sts(handler)
            for baudrate in baudrates:
                if not handler.setBaudRate(baudrate):
                    continue
                for sid in range(254):
                    model, result, error = packet.ping(sid)
                    if result == COMM_SUCCESS and not error:
                        found.append(
                            {
                                "port": info.device,
                                "servo_id": sid,
                                "baudrate": baudrate,
                                "model": model,
                            }
                        )
        finally:
            if opened:
                handler.closePort()
    return found


def discover_servos(baudrates: tuple[int, ...] = DEFAULT_BAUDRATES) -> list[dict]:
    if not baudrates:
        return []
    found = _scan((baudrates[0],))
    return found if found else _scan(baudrates[1:])


class _KeyReader:
    def __enter__(self):
        if os.name == "nt":
            import msvcrt

            self.read = msvcrt.getwch
        else:
            import termios, tty

            if not sys.stdin.isatty():
                raise GripperError("当前输入不是交互终端，无法使用 W/S。")
            self.termios, self.fd = termios, sys.stdin.fileno()
            self.settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.read = lambda: sys.stdin.read(1)
        return self

    def __exit__(self, *_):
        if os.name != "nt":
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.settings)


@dataclass
class _Channel:
    index: int
    config: dict
    bus: ServoBus
    calibrated: bool
    raw_previous: int | None = None
    steps: int = 0
    homed: bool = False
    target: int | None = None
    target_speed: int | None = None
    version: int = 0
    blocked_direction: str | None = None
    blocked_samples: int = 0
    last_command: float = 0
    stop_pending: bool = False
    stop_version: int = 0
    stop_result: tuple = ("就绪", "", None)
    stopped: threading.Event = field(default_factory=threading.Event)
    state: dict = field(default_factory=dict)
    next_status: float = 0
    test_active: bool = False
    test_collecting: bool = False
    test_version: int | None = None
    test_samples: list = field(default_factory=list)


class Gripper:
    def __init__(
        self,
        config_path: str | Path = DEFAULT_CONFIG_PATH,
        *,
        config: dict | None = None,
        allow_uninitialized=False,
    ):
        self.config_path = Path(config_path)
        self.config = {
            "servos": _servos(
                config if config is not None else _read_json(self.config_path)
            )
        }
        validate_config(self.config, calibrated=not allow_uninitialized)
        self.lock, self.stop_event, self.buses, self.channels = (
            threading.RLock(),
            threading.Event(),
            {},
            {},
        )
        try:
            for index, item in enumerate(self.config["servos"]):
                if item is None:
                    continue
                key = (item["serial_port"], item["baudrate"])
                bus = self.buses.setdefault(key, ServoBus(*key))
                self.channels[index] = _Channel(
                    index, item, bus, item.get("open_position_steps") is not None
                )
            for bus in self.buses.values():
                bus.open()
            for item in self.channels.values():
                sid = item.config["servo_id"]
                item.bus.verify(sid)
                item.bus.check_speed_mode(sid)
                item.bus.prepare(sid, item.config["grip_strength"])
                item.stopped.set()
                item.state = {
                    "state": "未回零",
                    "blocked_direction": None,
                    "target_width": None,
                    "width": None,
                    "telemetry": None,
                    "message": "请先使用 W/S 回到闭合参考位置，并按回车确认。",
                }
        except Exception:
            for bus in self.buses.values():
                bus.close()
            raise
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        if not self.stop_event.is_set():
            for item in self.channels.values():
                self._request_stop(item, "已停止", "夹爪已停止。")
            for item in self.channels.values():
                item.stopped.wait(0.5)
        self.stop_event.set()
        self.worker.join(timeout=1)
        for bus in self.buses.values():
            bus.close()

    def _channel(self, channel):
        if channel is None:
            if len(self.channels) == 1:
                return next(iter(self.channels.values()))
            raise GripperError("已配置多个舵机；请明确指定通道号 0 或 1。")
        if isinstance(channel, bool) or channel not in (0, 1):
            raise GripperError("通道号必须是 0 或 1。")
        if channel not in self.channels:
            raise GripperError(f"{channel} 号舵机未配置或不存在。")
        return self.channels[channel]

    @staticmethod
    def _width(item, steps):
        return max(0.0, min(1.0, steps / item.config["open_position_steps"]))

    @staticmethod
    def _thresholds(item):
        c = item.config
        return (
            (c["contact_load_threshold"], c["contact_current_ma"])
            if c.get("contact_load_threshold") is not None
            else (max(80, c["grip_strength"] * 6), 200 + c["grip_strength"] * 10)
        )

    def get_state(self, channel=None):
        item = self._channel(channel)
        with self.lock:
            return dict(item.state)

    def get_width(self, channel=None):
        item = self._channel(channel)
        if not item.homed or not item.calibrated:
            raise GripperError(f"{item.index} 号舵机尚未回零或标定。")
        return self._width(item, item.steps)

    def _request_stop(self, item, state, message, *, blocked_direction=None):
        with self.lock:
            item.version += 1
            item.target = item.target_speed = None
            item.blocked_samples = 0
            item.stop_pending, item.stop_version, item.stop_result = (
                True,
                item.version,
                (state, message, blocked_direction),
            )
            item.stopped.clear()
            item.state.update(
                state="停止中",
                blocked_direction=blocked_direction,
                target_width=None,
                message="正在停止舵机。",
            )

    def _finish_stop(self, item, version):
        with self.lock:
            if not item.stop_pending or item.stop_version != version:
                return
            item.stop_pending = False
            if item.version == version:
                state, message, blocked = item.stop_result
                item.state.update(
                    state=state,
                    blocked_direction=blocked,
                    target_width=None,
                    message=message,
                )
            item.stopped.set()

    def _set_target(self, item, target, label=None, speed=None):
        with self.lock:
            direction = (
                "opening"
                if (target - item.steps) * (item.config.get("open_position_steps") or 1)
                > 0
                else "closing"
            )
            if item.blocked_direction == direction and abs(target - item.steps) > 12:
                return False
            if item.blocked_direction != direction:
                item.blocked_direction = None
            item.version += 1
            item.target, item.target_speed, item.blocked_samples = target, speed, 0
            item.state.update(
                state="运动中",
                blocked_direction=None,
                target_width=label,
                message="正在移动。",
            )
            return True

    @staticmethod
    def _motion_speed(item, remaining, maximum):
        slow = max(80, round(maximum / item.config["control_frequency_hz"] * 4))
        if remaining >= slow:
            return maximum
        return max(
            min(maximum, max(30, min(150, maximum // 10))),
            round(maximum * remaining / slow),
        )

    def _update(self, item, now):
        try:
            t = item.bus.telemetry(item.config["servo_id"])
            with self.lock:
                if item.raw_previous is None:
                    item.raw_previous = t["raw_position"]
                else:
                    item.steps += _raw_delta(item.raw_previous, t["raw_position"])
                    item.raw_previous = t["raw_position"]
                steps, target, version, pending, stop_version = (
                    item.steps,
                    item.target,
                    item.version,
                    item.stop_pending,
                    item.stop_version,
                )
                item.state.update(
                    width=(
                        self._width(item, steps)
                        if item.homed and item.calibrated
                        else None
                    ),
                    telemetry=t,
                )
                if (
                    item.test_active
                    and item.test_collecting
                    and item.test_version == version
                    and target is not None
                ):
                    item.test_samples = (
                        item.test_samples
                        + [{"load": abs(t["load"]), "current_ma": abs(t["current_ma"])}]
                    )[-25:]
            if pending:
                item.bus.set_speed(item.config["servo_id"], 0)
                self._finish_stop(item, stop_version)
                return
            if t["temperature_c"] >= 70:
                self._request_stop(item, "异常", "舵机温度过高，已停止运动。")
                return
            if target is None:
                return
            direction = (
                "opening"
                if (target - steps) * (item.config.get("open_position_steps") or 1) > 0
                else "closing"
            )
            high = (
                abs(t["load"]) >= self._thresholds(item)[0]
                and abs(t["current_ma"]) >= self._thresholds(item)[1]
            )
            item.blocked_samples = item.blocked_samples + 1 if high else 0
            remaining = abs(target - steps)
            if item.blocked_samples >= 3:
                item.blocked_direction = direction
                self._request_stop(
                    item,
                    "已阻塞",
                    f"{direction} 方向检测到物体或物理限位，已停止继续施力。",
                    blocked_direction=direction,
                )
            elif remaining <= 12:
                self._request_stop(item, "已到位", "已到达目标位置。")
            elif now - item.last_command >= 1 / item.config["control_frequency_hz"]:
                speed = self._motion_speed(
                    item, remaining, item.target_speed or item.config["speed"]
                )
                item.bus.set_speed(
                    item.config["servo_id"], speed if target > steps else -speed
                )
                item.last_command = now
                if item.test_active and item.test_version == version:
                    item.test_collecting = True
                item.state.update(
                    state="运动中", blocked_direction=None, message="正在移动。"
                )
        except GripperError as exc:
            with self.lock:
                item.target = item.target_speed = None
                item.stop_pending = False
                item.stopped.set()
                item.state.update(state="异常", message=str(exc))

    def _run(self):
        while not self.stop_event.is_set():
            now, next_due = time.monotonic(), time.monotonic() + 0.05
            for item in self.channels.values():
                if now >= item.next_status:
                    self._update(item, now)
                    item.next_status = now + 1 / item.config["status_frequency_hz"]
                next_due = min(next_due, item.next_status)
            self.stop_event.wait(max(0.001, next_due - time.monotonic()))

    def goto(self, width, channel=None):
        if not isinstance(width, (int, float)) or not 0 <= float(width) <= 1:
            raise ValueError("goto() 的取值必须在 0 到 1 之间。")
        item = self._channel(channel)
        if not item.homed or not item.calibrated:
            raise GripperError(f"{item.index} 号舵机请先调用 home()。")
        self._set_target(
            item, round(item.config["open_position_steps"] * float(width)), float(width)
        )

    def wait(self, timeout=None, channel=None):
        item = self._channel(channel)
        deadline = time.monotonic() + (timeout or item.config["motion_timeout_s"])
        while time.monotonic() < deadline:
            result = self.get_state(item.index)
            if result["state"] not in {"运动中", "停止中", "未回零"}:
                return result
            time.sleep(0.02)
        raise GripperError(f"{item.index} 号舵机等待动作完成超时。")

    def reset(self, channel=None):
        item = self._channel(channel)
        if not item.homed or not item.calibrated:
            raise GripperError(f"{item.index} 号舵机请先调用 home()。")
        self._set_target(
            item,
            item.config["neutral_position_steps"],
            self._width(item, item.config["neutral_position_steps"]),
        )
        result = self.wait(channel=item.index)
        if result["state"] != "已到位":
            raise GripperError("复位失败：" + result["message"])

    def manual_step(self, direction, channel=None):
        if direction not in {-1, 1}:
            raise ValueError("手动方向只能为 -1 或 1。")
        item = self._channel(channel)
        ok = self._set_target(
            item,
            item.steps + direction * item.config["calibration_step_steps"],
            speed=min(item.config["speed"], 300),
        )
        if ok and item.test_active:
            item.test_samples = []
            item.test_version = item.version
            item.test_collecting = False
        return ok

    def _keyboard(self, item, name, home):
        with _KeyReader() as reader:
            while True:
                key = reader.read().lower()
                if key in {"w", "s"}:
                    if not self.manual_step(1 if key == "w" else -1, item.index):
                        print(
                            "该方向刚检测到阻塞；请改按相反方向，或按回车确认当前位置。"
                        )
                elif key in {"\r", "\n"}:
                    self._request_stop(item, "就绪", "已停止在当前位置。")
                    item.stopped.wait(1)
                    if home:
                        item.steps, item.homed, item.blocked_direction = 0, True, None
                        item.state.update(
                            state="就绪", width=0.0, message="已确认闭合参考位置。"
                        )
                    print(f"已确认{name}。\n")
                    return True
                elif key == "q":
                    self._request_stop(item, "就绪", "用户已取消手动移动。")
                    item.stopped.wait(1)
                    print("已取消。\n")
                    return False

    def home(self, channel=None):
        item = self._channel(channel)
        print(
            f"\n{item.index} 号舵机回零：W/S 控制移动。到闭合参考位置后按回车；Q 退出。"
        )
        return self._keyboard(item, "闭合参考位置", True)

    def calibrate_position(self, name, channel=None):
        item = self._channel(channel)
        print(f"\n标定 {item.index} 号舵机{name}：W/S 控制移动。到位后按回车；Q 退出。")
        return item.steps if self._keyboard(item, name, False) else None

    def guided_grip_test(self, channel=None):
        item = self._channel(channel)
        if not item.homed or not item.calibrated:
            raise GripperError("请先完成闭合、中立和张开位置标定。")
        close = -1 if item.config["open_position_steps"] > 0 else 1
        close_key, open_key = ("S", "W") if close < 0 else ("W", "S")
        print(
            f"\n{item.index} 号舵机可选夹紧测试：放入测试物。{close_key} 闭合、{open_key} 张开；回车记录，Q 跳过。"
        )
        item.test_active, item.test_samples, item.test_version, item.test_collecting = (
            True,
            [],
            None,
            False,
        )
        with _KeyReader() as reader:
            while True:
                key = reader.read().lower()
                if key == close_key.lower():
                    self.manual_step(close, item.index)
                elif key == open_key.lower():
                    self.manual_step(-close, item.index)
                elif key in {"\r", "\n", "q"}:
                    samples = [
                        x for x in item.test_samples if x["load"] and x["current_ma"]
                    ][-5:]
                    item.test_active = item.test_collecting = False
                    self._request_stop(item, "就绪", "夹紧测试已停止。")
                    item.stopped.wait(1)
                    if key == "q":
                        print("已跳过夹紧测试。\n")
                        return None
                    if not samples:
                        return {"load": 0, "current_ma": 0}
                    return {
                        "load": round(sum(x["load"] for x in samples) / len(samples)),
                        "current_ma": round(
                            sum(x["current_ma"] for x in samples) / len(samples)
                        ),
                    }
