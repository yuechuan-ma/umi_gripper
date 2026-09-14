"""通过 W/S 分别完成 0、1 号夹爪舵机标定。"""

from gripper import (
    DEFAULT_CONFIG_PATH,
    Gripper,
    GripperError,
    ServoBus,
    default_servo_config,
    discover_servos,
    save_config,
    validate_config,
)


def choose(candidates, channel, forbidden=None):
    print(f"\n选择 {channel} 号舵机：")
    print("  0. 本通道为空")
    for number, item in enumerate(candidates, 1):
        print(
            f"  {number}. 串口 {item['port']}，ID {item['servo_id']}，速度 {item['baudrate']}"
        )
    while True:
        answer = input("输入序号，或输入 q 退出：").strip().lower()
        if answer == "q":
            return "quit"
        if answer == "0":
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(candidates):
            item = candidates[int(answer) - 1]
            if forbidden == (item["port"], item["servo_id"]):
                print("同一串口和 ID 不能分配给两个通道，请重新选择。")
            else:
                return item
        else:
            print("序号无效，请重新输入。")


def optional_grip_test(gripper, config, channel):
    if (
        input(f"{channel} 号三点标定完成。按回车进行可选夹紧测试；输入 q 跳过：")
        .strip()
        .lower()
        == "q"
    ):
        print("已跳过夹紧测试。")
        return
    strength = gripper.guided_grip_test(channel)
    if strength is None:
        return
    config["grip_strength"] = strength
    print("已保存夹紧力度。")


def completed_config(config):
    position_keys = (
        "closed_position_steps",
        "neutral_position_steps",
        "open_position_steps",
    )
    servos = [
        item
        if item is not None and all(item.get(key) is not None for key in position_keys)
        else None
        for item in config["servos"]
    ]
    saved = {"servos": servos}
    validate_config(saved)
    return saved


def configure_position_mode(config):
    buses = {}
    try:
        for item in config["servos"]:
            if item is None:
                continue
            buses.setdefault(
                (item["serial_port"], item["baudrate"]),
                ServoBus(item["serial_port"], item["baudrate"]),
            )
        for bus in buses.values():
            bus.open()
        for item in config["servos"]:
            if item is not None:
                bus = buses[(item["serial_port"], item["baudrate"])]
                bus.configure_position_mode(item["servo_id"])
                bus.set_current_position_as_middle(item["servo_id"])
    finally:
        for bus in buses.values():
            bus.close()


def main():
    print("夹爪初始化：请确认跳线在 B 位置。")
    if input("按回车扫描串口和 ID；输入 q 退出：").strip().lower() == "q":
        return
    candidates = discover_servos()
    if not candidates:
        print("未找到舵机。请检查 USB、供电、跳线和舵机线。")
        return
    first = choose(candidates, 0)
    if first == "quit":
        return
    second = choose(
        candidates, 1, None if first is None else (first["port"], first["servo_id"])
    )
    if second == "quit":
        return
    config = {
        "servos": [
            default_servo_config(first) if first else None,
            default_servo_config(second) if second else None,
        ]
    }
    try:
        validate_config(config, calibrated=False)
        configure_position_mode(config)
        with Gripper(config=config, allow_uninitialized=True) as gripper:
            print(
                "已切换为单圈绝对位置模式，并将各舵机当前安装角度设为中位。"
                "每个通道依次使用 W/S 标定。"
            )
            for channel, item in enumerate(config["servos"]):
                if item is None:
                    continue
                closed = gripper.calibrate_position("闭合位置", channel)
                if closed is None:
                    return
                neutral = gripper.calibrate_position("中立位置", channel)
                if neutral is None:
                    return
                opened = gripper.calibrate_position("张开位置", channel)
                if opened is None:
                    return
                item["closed_position_steps"] = closed
                item["neutral_position_steps"] = neutral
                item["open_position_steps"] = opened
                gripper.channels[channel].config.update(item)
                gripper.channels[channel].calibrated = True
                optional_grip_test(gripper, item, channel)
                save_config(completed_config(config))
                print(f"{channel} 号舵机标定已保存。")
            validate_config(config)
        save_config(config)
        print("标定完成并已保存。之后创建 Gripper 对象会直接读取当前位置。")
    except GripperError as exc:
        print(f"初始化失败：{exc}")


if __name__ == "__main__":
    main()
