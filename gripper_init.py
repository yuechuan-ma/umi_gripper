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
    telemetry = gripper.guided_grip_test(channel)
    if telemetry is None:
        return
    load, current = abs(telemetry["load"]), abs(telemetry["current_ma"])
    if not load or not current:
        print("未得到有效的负载或电流反馈，不保存夹紧判定值。")
        return
    suggested_load, suggested_current = max(1, min(1000, round(load * 0.95))), max(
        1, min(5000, round(current * 0.95))
    )
    print(
        f"记录到：负载 {load}，电流 {current}mA。建议判定值：负载 {suggested_load}，电流 {suggested_current}mA。"
    )
    if input("确认已夹稳，输入 y 保存建议值；其他输入不保存：").strip().lower() == "y":
        config["contact_load_threshold"], config["contact_current_ma"] = (
            suggested_load,
            suggested_current,
        )
        print("已保存夹紧判定值。")


def configure_speed_mode(config):
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
                buses[(item["serial_port"], item["baudrate"])].configure_speed_mode(
                    item["servo_id"]
                )
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
        configure_speed_mode(config)
        with Gripper(config=config, allow_uninitialized=True) as gripper:
            print("已切换为多圈速度模式。每个通道依次使用 W/S 标定。")
            for channel, item in enumerate(config["servos"]):
                if item is None:
                    continue
                if not gripper.home(channel):
                    return
                item["closed_position_steps"] = 0
                neutral = gripper.calibrate_position("中立位置", channel)
                if neutral is None:
                    return
                opened = gripper.calibrate_position("张开位置", channel)
                if opened is None:
                    return
                item["neutral_position_steps"], item["open_position_steps"] = (
                    neutral,
                    opened,
                )
                gripper.channels[channel].config.update(item)
                gripper.channels[channel].calibrated = True
                optional_grip_test(gripper, item, channel)
            validate_config(config)
        save_config(config)
        print("标定完成并已保存。之后创建 Gripper 对象时，各通道都必须先调用 home()。")
    except GripperError as exc:
        print(f"初始化失败：{exc}")


if __name__ == "__main__":
    main()
