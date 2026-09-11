"""支持多圈传动、单后台循环的单舵机夹爪控制。"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
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


def _integer(config: dict, key: str, low: int, high: int) -> int:
    value = config.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not low <= value <= high
    ):
        raise ConfigError(f"配置项“{key}”必须是 {low} 到 {high} 的整数。")
    return value


def validate_config(config: dict, *, calibrated: bool = True) -> None:
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
        raise ConfigError("配置尚未完成：" + "、".join(missing) + "。")
    if not isinstance(config["serial_port"], str):
        raise ConfigError("配置项“serial_port”必须是串口名称。")
    
    _integer(config, "servo_id", 0, 253)
    _integer(config, "baudrate", 9600, 1_000_000)
    _integer(config, "speed", 1, 3400)
    _integer(config, "grip_strength", 1, 100)

    control = _integer(config, "control_frequency_hz", 1, 100)
    if _integer(config, "status_frequency_hz", 1, 100) < control:
        raise ConfigError("后台状态频率不能低于控制频率。")
    
    _integer(config, "calibration_step_steps", 10, 1024)
    _integer(config, "motion_timeout_s", 2, 120)

    load, current = config.get("contact_load_threshold"), config.get("contact_current_ma")
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
        raise ConfigError("闭合参考位置必须为 0；请重新初始化。")
    opened, neutral = config["open_position_steps"], config["neutral_position_steps"]
    if abs(opened) < 10:
        raise ConfigError("张开行程不合理；请重新初始化。")
    if opened * neutral < 0 or abs(neutral) > abs(opened):
        raise ConfigError("中立位置必须位于闭合和张开位置之间。")


def _raw_delta(previous: int, current: int) -> int:
    delta = (current - previous) % STEPS_PER_TURN
    return delta - STEPS_PER_TURN if delta > STEPS_PER_TURN // 2 else delta


class ServoBus:
    def __init__(self, port: str, servo_id: int, baudrate: int):
        self.port_name, self.servo_id, self.baudrate = port, servo_id, baudrate
        self.port, self.packet = PortHandler(port), None

    @staticmethod
    def _check(result: int, error: int, action: str) -> None:
        if result != COMM_SUCCESS:
            raise GripperError(f"{action}失败：串口通信异常（代码 {result}）。")
        if error:
            raise GripperError(f"{action}失败：舵机报告异常（代码 {error}）。")

    def open(self) -> None:
        if not self.port.openPort():
            raise GripperError(
                f"无法打开串口 {self.port_name}。请确认设备已连接且未被占用。"
            )
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise GripperError(f"无法设置串口速度 {self.baudrate}。")
        self.packet = sms_sts(self.port)
        _, result, error = self.packet.ping(self.servo_id)
        self._check(result, error, "确认舵机")

    def close(self) -> None:
        self.port.closePort()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def _read1(self, address: int, action: str) -> int:
        value, result, error = self.packet.read1ByteTxRx(self.servo_id, address)
        self._check(result, error, action)
        return value

    def _read2(self, address: int, action: str) -> int:
        value, result, error = self.packet.read2ByteTxRx(self.servo_id, address)
        self._check(result, error, action)
        return value

    def _write1(self, address: int, value: int, action: str) -> None:
        result, error = self.packet.write1ByteTxRx(self.servo_id, address, value)
        self._check(result, error, action)

    def _write2(self, address: int, value: int, action: str) -> None:
        result, error = self.packet.write2ByteTxRx(self.servo_id, address, value)
        self._check(result, error, action)

    def configure_speed_mode(self) -> None:
        self._write1(SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")
        self._write1(55, 0, "解锁舵机设置")
        self._write2(9, 0, "设置最小角度")
        self._write2(11, 0, "设置最大角度")
        self._write1(33, 1, "切换多圈速度模式")
        self._write1(55, 1, "锁定舵机设置")

    def check_speed_mode(self) -> None:
        if self._read1(33, "读取舵机模式") != 1:
            raise GripperError("舵机未处于多圈速度模式。请先运行 gripper_init.py。")

    def prepare(self, strength: int) -> None:
        self._write2(48, strength * 10, "设置夹紧力度")
        self._write1(SMS_STS_TORQUE_ENABLE, 1, "开启扭矩")

    def set_speed(self, speed: int) -> None:
        """设定有符号转速；速度为 0 是明确停止命令。"""
        result, error = self.packet.WriteSpec(self.servo_id, speed, 50)
        self._check(result, error, "下发转速")

    def stop_motion(self) -> None:
        self.set_speed(0)

    def telemetry(self) -> dict:
        position, result, error = self.packet.ReadPos(self.servo_id)
        self._check(result, error, "读取当前位置")
        # 多圈模式下部分固件会返回带圈数的有符号位置；后台自行累计圈数，故只保留单圈余数。
        position %= STEPS_PER_TURN
        load = self._read2(SMS_STS_PRESENT_LOAD_L, "读取负载")
        current = self._read2(SMS_STS_PRESENT_CURRENT_L, "读取电流")
        temperature = self._read1(SMS_STS_PRESENT_TEMPERATURE, "读取温度")
        load = -(load & ~(1 << 10)) if load & (1 << 10) else load
        current = -(current & ~(1 << 15)) if current & (1 << 15) else current
        return {
            "raw_position": position,
            "load": load,
            "current_ma": round(current * 6.5),
            "temperature_c": temperature,
        }


def _scan(baudrates: tuple[int, ...]) -> list[dict]:
    found = []
    for info in list_ports.comports():
        handler = PortHandler(info.device)
        try:
            if not handler.openPort():
                continue
            packet = sms_sts(handler)
            for baudrate in baudrates:
                if not handler.setBaudRate(baudrate):
                    continue
                for servo_id in range(254):
                    model, result, error = packet.ping(servo_id)
                    if result == COMM_SUCCESS and not error:
                        found.append(
                            {
                                "port": info.device,
                                "servo_id": servo_id,
                                "baudrate": baudrate,
                                "model": model,
                            }
                        )
        finally:
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


class Gripper:
    """创建后调用 home()；goto() 只更新目标，所有串口收发均由后台循环完成。"""

    def __init__(
        self,
        config_path: str | Path = DEFAULT_CONFIG_PATH,
        *,
        config: dict | None = None,
        allow_uninitialized: bool = False,
    ):
        self.config_path = Path(config_path)
        self.config = (
            dict(config) if config is not None else _read_json(self.config_path)
        )

        validate_config(self.config, calibrated=not allow_uninitialized)
        self.calibrated = self.config.get("open_position_steps") is not None

        self.bus = ServoBus(
            self.config["serial_port"], self.config["servo_id"], self.config["baudrate"]
        )
        self.bus.open()
        self.bus.check_speed_mode()
        self.bus.prepare(self.config["grip_strength"])

        self.lock, self.stop_event = threading.RLock(), threading.Event()
        self.motion_stopped = threading.Event()
        self.motion_stopped.set()

        self.raw_previous, self.steps, self.homed = None, 0, False
        self.target, self.target_speed, self.command_version = None, None, 0
        self.blocked_direction, self.blocked_samples, self.last_command = None, 0, 0.0
        self.stop_pending, self.stop_version, self.stop_result = False, 0, None

        self.grip_test_active = False
        self.grip_test_samples = []
        self.grip_test_version = None
        self.grip_test_collecting = False

        self.state = {
            "state": "未回零",
            "blocked_direction": None,
            "target_width": None,
            "width": None,
            "telemetry": None,
            "message": "请先使用 W/S 回到闭合参考位置，并按回车确认。",
        }

        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self) -> None:
        if not self.stop_event.is_set():
            self._request_stop("已停止", "夹爪已停止。")
            self.motion_stopped.wait(0.5)
        self.stop_event.set()
        self.worker.join(timeout=1)
        self.bus.close()

    def _set_state(self, **changes):
        with self.lock:
            self.state.update(changes)

    def get_state(self) -> dict:
        with self.lock:
            return dict(self.state)

    def _thresholds(self):
        if self.config.get("contact_load_threshold") is not None:
            return (
                self.config["contact_load_threshold"],
                self.config["contact_current_ma"],
            )

        # 以下为人工设定的保守赋值
        strength = self.config["grip_strength"]
        return max(80, strength * 6), 200 + strength * 10

    def _width(self, steps: int) -> float:
        return max(0.0, min(1.0, steps / self.config["open_position_steps"]))

    def get_width(self) -> float:
        with self.lock:
            if not self.homed or not self.calibrated:
                raise GripperError("尚未回零或标定，无法获取夹爪宽度。")
            return self._width(self.steps)

    def _request_stop(
        self, state: str, message: str, *, blocked_direction: str | None = None
    ) -> None:
        """请求后台发送零速度；调用方不直接操作串口。"""
        with self.lock:
            self.command_version += 1
            self.target, self.target_speed, self.blocked_samples = None, None, 0
            self.stop_pending, self.stop_version = True, self.command_version
            self.stop_result = (state, message, blocked_direction)
            self.motion_stopped.clear()
            self.state.update(
                state="停止中",
                blocked_direction=blocked_direction,
                target_width=None,
                message="正在停止舵机。",
            )

    def _finish_stop(self, version: int) -> None:
        with self.lock:
            if not self.stop_pending or self.stop_version != version:
                return
            self.stop_pending = False
            state, message, blocked_direction = self.stop_result
            if self.command_version == version:
                self.state.update(
                    state=state,
                    blocked_direction=blocked_direction,
                    target_width=None,
                    message=message,
                )
            self.motion_stopped.set()

    def _wait_stopped(self, timeout: float = 1.0) -> None:
        if not self.motion_stopped.wait(timeout):
            raise GripperError(
                "停止命令未能在 1 秒内确认发送；请立即断开舵机外部电源。"
            )

    def _motion_speed(self, remaining_steps: int, maximum_speed: int) -> int:
        """离目标越近，速度越低，避免越过目标后反复修正。"""
        slowdown_distance = max(
            80,
            round(maximum_speed / self.config["control_frequency_hz"] * 4),
        )
        if remaining_steps >= slowdown_distance:
            return maximum_speed
        minimum_speed = min(maximum_speed, max(30, min(150, maximum_speed // 10)))
        return max(
            minimum_speed,
            round(maximum_speed * remaining_steps / slowdown_distance),
        )

    def _run(self) -> None:
        interval, command_interval = (
            1 / self.config["status_frequency_hz"],
            1 / self.config["control_frequency_hz"],
        )
        load_limit, current_limit = self._thresholds()
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                telemetry = self.bus.telemetry()
                with self.lock:
                    raw = telemetry["raw_position"]
                    if self.raw_previous is None:
                        self.raw_previous = raw
                    else:
                        self.steps += _raw_delta(self.raw_previous, raw)
                        self.raw_previous = raw
                    steps, target, target_speed, homed, version = (
                        self.steps,
                        self.target,
                        self.target_speed,
                        self.homed,
                        self.command_version,
                    )
                    stop_pending, stop_version = self.stop_pending, self.stop_version
                    if (
                        self.grip_test_active
                        and self.grip_test_collecting
                        and self.grip_test_version == version
                        and target is not None
                    ):
                        self.grip_test_samples.append(
                            {
                                "load": abs(telemetry["load"]),
                                "current_ma": abs(telemetry["current_ma"]),
                            }
                        )
                        self.grip_test_samples = self.grip_test_samples[-25:]
                width = self._width(steps) if homed and self.calibrated else None
                self._set_state(width=width, telemetry=telemetry)
                if stop_pending:
                    self.bus.stop_motion()
                    self._finish_stop(stop_version)
                    continue
                if telemetry["temperature_c"] >= 70:
                    self._request_stop("异常", "舵机温度过高，已停止运动。")
                    continue
                if target is not None:
                    with self.lock:
                        if version != self.command_version:
                            continue
                    opening = (target - steps) * (
                        self.config.get("open_position_steps") or 1
                    ) > 0
                    direction = "opening" if opening else "closing"
                    remaining = abs(target - steps)
                    high = (
                        abs(telemetry["load"]) >= load_limit
                        and abs(telemetry["current_ma"]) >= current_limit
                    )
                    self.blocked_samples = self.blocked_samples + 1 if high else 0
                    if self.blocked_samples >= 3:
                        with self.lock:
                            if version != self.command_version:
                                continue
                            self.blocked_direction = direction
                        self._request_stop(
                            "已阻塞",
                            f"{direction} 方向检测到物体或物理限位，已停止继续施力。",
                            blocked_direction=direction,
                        )
                    elif remaining <= 12:
                        with self.lock:
                            if version != self.command_version:
                                continue
                        self._request_stop("已到位", "已到达目标位置。")
                    elif started - self.last_command >= command_interval:
                        with self.lock:
                            if version != self.command_version:
                                continue
                        maximum_speed = target_speed or self.config["speed"]
                        speed = self._motion_speed(remaining, maximum_speed)
                        self.bus.set_speed(speed if target > steps else -speed)
                        self.last_command = started
                        with self.lock:
                            if (
                                self.grip_test_active
                                and self.grip_test_version == version
                            ):
                                self.grip_test_collecting = True
                        self._set_state(
                            state="运动中", blocked_direction=None, message="正在移动。"
                        )
            except GripperError as exc:
                with self.lock:
                    self.target, self.target_speed, self.stop_pending = (
                        None,
                        None,
                        False,
                    )
                    self.motion_stopped.set()
                self._set_state(state="异常", message=str(exc))
            self.stop_event.wait(max(0, interval - (time.monotonic() - started)))

    def _set_target(self, target: int, label=None, *, speed: int | None = None) -> bool:
        with self.lock:
            direction = (
                "opening"
                if (target - self.steps) * (self.config.get("open_position_steps") or 1)
                > 0
                else "closing"
            )
            if self.blocked_direction == direction and abs(target - self.steps) > 12:
                return False
            if self.blocked_direction != direction:
                self.blocked_direction = None
            self.command_version += 1
            self.target, self.target_speed, self.blocked_samples = target, speed, 0
            self.state.update(
                state="运动中",
                blocked_direction=None,
                target_width=label,
                message="正在移动。",
            )
            return True

    def goto(self, width: float) -> None:
        if not isinstance(width, (int, float)) or not 0 <= float(width) <= 1:
            raise ValueError("goto() 的取值必须在 0 到 1 之间。")
        with self.lock:
            if not self.homed or not self.calibrated:
                raise GripperError("请先调用 home() 回到闭合参考位置。")
        self._set_target(
            round(self.config["open_position_steps"] * float(width)), float(width)
        )

    def reset(self) -> None:
        with self.lock:
            if not self.homed or not self.calibrated:
                raise GripperError("请先调用 home() 回到闭合参考位置。")
        self._set_target(
            self.config["neutral_position_steps"],
            self._width(self.config["neutral_position_steps"]),
        )
        result = self.wait()
        if result["state"] != "已到位":
            raise GripperError("复位失败：" + result["message"])

    def wait(self, timeout=None) -> dict:
        deadline = time.monotonic() + (timeout or self.config["motion_timeout_s"])
        while time.monotonic() < deadline:
            result = self.get_state()
            if result["state"] not in {"运动中", "停止中", "未回零"}:
                return result
            time.sleep(0.02)
        raise GripperError("等待夹爪动作完成超时。")

    def manual_step(self, direction: int) -> bool:
        if direction not in {-1, 1}:
            raise ValueError("手动方向只能为 -1 或 1。")
        with self.lock:
            base = self.steps
        accepted = self._set_target(
            base + direction * self.config["calibration_step_steps"],
            speed=min(self.config["speed"], 300),
        )
        if accepted:
            with self.lock:
                if self.grip_test_active:
                    self.grip_test_samples = []
                    self.grip_test_version = self.command_version
                    self.grip_test_collecting = False
        return accepted

    def _finish_grip_test_sampling(self) -> dict:
        with self.lock:
            self.grip_test_active = False
            self.grip_test_collecting = False
            samples = [
                sample
                for sample in self.grip_test_samples
                if sample["load"] > 0 and sample["current_ma"] > 0
            ][-5:]
        if not samples:
            return {"load": 0, "current_ma": 0}
        return {
            "load": round(sum(sample["load"] for sample in samples) / len(samples)),
            "current_ma": round(
                sum(sample["current_ma"] for sample in samples) / len(samples)
            ),
        }

    def guided_grip_test(self) -> dict | None:
        """在标定完成后，由用户逐步闭合并记录一次实际夹取时的反馈。"""
        if not self.homed or not self.calibrated:
            raise GripperError("请先完成闭合、中立和张开位置标定。")

        open_position = self.config["open_position_steps"]
        close_direction = -1 if open_position > 0 else 1
        close_key = "S" if close_direction < 0 else "W"
        open_key = "W" if close_direction < 0 else "S"
        print(
            "\n可选夹紧测试：请将有代表性的测试物放入已张开的夹爪中。"
            "确保可随时断电，手和工具远离夹爪。"
        )
        print(
            f"{close_key} 为闭合、{open_key} 为张开；每按一次只低速移动一小段。"
            "确认夹稳后按回车记录反馈；Q 会立即停止并跳过测试。"
        )
        with self.lock:
            self.grip_test_active = True
            self.grip_test_samples = []
            self.grip_test_version = None
            self.grip_test_collecting = False

        with _KeyReader() as reader:
            while True:
                key = reader.read().lower()
                if key == close_key.lower():
                    if not self.manual_step(close_direction):
                        print("已检测到阻塞；请确认物体是否夹稳，或按相反方向松开。")
                elif key == open_key.lower():
                    self.manual_step(-close_direction)
                elif key in {"\r", "\n"}:
                    telemetry = self._finish_grip_test_sampling()
                    self._request_stop("就绪", "夹紧测试已停止在当前位置。")
                    self._wait_stopped()
                    print("已记录当前反馈。\n")
                    return telemetry
                elif key == "q":
                    with self.lock:
                        self.grip_test_active = False
                        self.grip_test_samples = []
                        self.grip_test_collecting = False
                    self._request_stop("就绪", "用户已跳过夹紧测试。")
                    self._wait_stopped()
                    print("已跳过夹紧测试。\n")
                    return None

    def confirm_home(self) -> None:
        with self.lock:
            self.steps, self.homed, self.blocked_direction = 0, True, None
            self.state.update(
                state="就绪",
                blocked_direction=None,
                width=0.0,
                message="已确认闭合参考位置。",
            )

    def _keyboard_position(self, name: str, home: bool) -> bool:
        with _KeyReader() as reader:
            while True:
                key = reader.read().lower()
                if key == "w":
                    if not self.manual_step(1):
                        print("该方向刚检测到阻塞；请改按 S，或按回车确认当前位置。")
                elif key == "s":
                    if not self.manual_step(-1):
                        print("该方向刚检测到阻塞；请改按 W，或按回车确认当前位置。")
                elif key in {"\r", "\n"}:
                    self._request_stop("就绪", "已停止在当前位置。")
                    self._wait_stopped()
                    if home:
                        self.confirm_home()
                    print(f"已确认{name}。\n")
                    return True
                elif key == "q":
                    self._request_stop("就绪", "用户已取消手动移动。")
                    self._wait_stopped()
                    print("已取消。\n")
                    return False

    def home(self) -> bool:
        print(
            "\n回零：W 为正向，S 为反向；每按一次只低速走一小段。到闭合参考位置后按回车；Q 会立即停止并退出。"
        )
        return self._keyboard_position("闭合参考位置", True)

    def calibrate_position(self, name: str) -> int | None:
        print(
            f"\n标定{name}：W 为正向，S 为反向；每按一次只低速走一小段。到位后按回车；Q 会立即停止并退出。"
        )
        if not self._keyboard_position(name, False):
            return None
        with self.lock:
            return self.steps
