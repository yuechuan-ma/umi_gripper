"""手动核对和修改总线舵机 ID。"""

from gripper import GripperError, ServoBus, discover_servos


def choose(candidates):
    for number, item in enumerate(candidates, 1):
        print(
            f"  {number}. 串口 {item['port']}，ID {item['servo_id']}，速度 {item['baudrate']}"
        )
    while True:
        answer = input("输入序号，或 q 退出：").strip().lower()
        if answer == "q":
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(candidates):
            return candidates[int(answer) - 1]
        print("序号无效，请重新输入。")


def main():
    print("舵机 ID 工具：请确认跳线在 B 位置。")
    print("修改 ID 时，同一串口只能连接目标舵机；同 ID 舵机会一起被修改。")
    while True:
        if input("按回车扫描；输入 q 退出：").strip().lower() == "q":
            return
        candidates = discover_servos()
        if not candidates:
            print("未找到舵机。请检查 USB、供电、跳线和舵机线。")
            continue
        selected = choose(candidates)
        if selected is None:
            return
        print(
            f"当前：串口 {selected['port']}，ID {selected['servo_id']}，版本编号 {selected['model']}。"
        )
        answer = (
            input("输入新 ID（0-253），直接回车仅核对，或 q 返回扫描：").strip().lower()
        )
        if not answer:
            continue
        if answer == "q":
            continue
        if not answer.isdigit() or not 0 <= int(answer) <= 253:
            print("ID 无效，请输入 0 到 253 的整数。")
            continue
        new_id = int(answer)
        if new_id == selected["servo_id"]:
            print("新 ID 与当前 ID 相同，无需修改。")
            continue
        if any(
            item["port"] == selected["port"] and item["servo_id"] == new_id
            for item in candidates
        ):
            print("该串口上已发现此 ID。请重新输入新 ID 或退出。")
            continue
        try:
            with ServoBus(selected["port"], selected["baudrate"]) as bus:
                bus.verify(selected["servo_id"])
                bus.change_id(selected["servo_id"], new_id)
            print(
                f"ID 已从 {selected['servo_id']} 修改为 {new_id}，且已重新确认通信正常。"
            )
        except GripperError as exc:
            print(f"修改失败：{exc}")


if __name__ == "__main__":
    main()
