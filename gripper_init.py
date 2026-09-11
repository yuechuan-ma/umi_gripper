"""通过 W/S 完成多圈夹爪标定。"""

from gripper import (
    DEFAULT_CONFIG_PATH,
    Gripper,
    GripperError,
    ServoBus,
    discover_servos,
    save_config,
    validate_config,
)


def choose(candidates):
    print("找到以下候选项：")
    for index, item in enumerate(candidates, 1):
        print(
            f"  {index}. 串口 {item['port']}，ID {item['servo_id']}，速度 {item['baudrate']}"
        )
    while True:
        answer = input("输入序号，或输入 q 退出：").strip().lower()
        if answer == "q":
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(candidates):
            return candidates[int(answer) - 1]
        print("序号无效，请重新输入。")


def optional_grip_test(gripper, config):
    answer = (
        input("三点标定完成。按回车进行可选夹紧测试；输入 q 跳过：").strip().lower()
    )
    if answer == "q":
        print("已跳过夹紧测试。")
        return

    telemetry = gripper.guided_grip_test()
    if telemetry is None:
        return

    load = abs(telemetry["load"])
    current = abs(telemetry["current_ma"])
    if load == 0 or current == 0:
        print("未得到有效的负载或电流反馈，不保存夹紧判定值。")
        return

    suggested_load = max(1, min(1000, round(load * 0.95)))
    suggested_current = max(1, min(5000, round(current * 0.95)))
    print(
        f"记录到：负载 {load}，电流 {current}mA。\n"
        f"建议判定值：负载 {suggested_load}，电流 {suggested_current}mA。"
    )
    answer = (
        input("确认测试物已夹稳，输入 y 保存建议值；直接回车或输入 q 则不保存：")
        .strip()
        .lower()
    )
    if answer == "y":
        config["contact_load_threshold"] = suggested_load
        config["contact_current_ma"] = suggested_current
        print("已保存夹紧判定值。")
    else:
        print("未保存夹紧判定值，将继续使用默认判定方式。")


def main():
    print("夹爪初始化：请确认跳线在 B 位置。")
    if input("按回车扫描串口和 ID；输入 q 退出：").strip().lower() == "q":
        return
    candidates = discover_servos()
    if not candidates:
        print("未找到舵机。请检查 USB、供电、跳线和舵机线。")
        return
    selected = choose(candidates)
    if selected is None:
        return
    config = {
        "serial_port": selected["port"],
        "servo_id": selected["servo_id"],
        "baudrate": selected["baudrate"],
        "servo_model_number": selected["model"],
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
    try:
        with ServoBus(
            config["serial_port"], config["servo_id"], config["baudrate"]
        ) as bus:
            bus.configure_speed_mode()
        with Gripper(config=config, allow_uninitialized=True) as gripper:
            print(
                "已切换为多圈速度模式。W/S 控制移动。"
            )
            if not gripper.home():
                return
            config["closed_position_steps"] = 0
            neutral = gripper.calibrate_position("中立位置")
            if neutral is None:
                return
            opened = gripper.calibrate_position("张开位置")
            if opened is None:
                return
            config["neutral_position_steps"], config["open_position_steps"] = (
                neutral,
                opened,
            )
            validate_config(config)
            gripper.config.update(config)
            gripper.calibrated = True
            optional_grip_test(gripper, config)
        save_config(config)
        print("标定完成并已保存。之后每次创建 Gripper 对象，都必须先调用 home()。")
    except GripperError as exc:
        print(f"初始化失败：{exc}")


if __name__ == "__main__":
    main()
