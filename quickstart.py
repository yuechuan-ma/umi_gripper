"""演示单通道或双通道夹爪控制。"""

from gripper import Gripper, GripperError


def main():
    try:
        gripper = Gripper()
        try:
            channels = sorted(gripper.channels)
            for channel in channels:
                print(f"{channel} 号舵机已连接，正在读取当前位置。")
            while True:
                choice = (
                    input("输入“开度 通道号”（例如 0.8 0），或 q 退出：")
                    .strip()
                    .lower()
                )
                if choice == "q":
                    return
                try:
                    width_text, channel_text = choice.split()
                    channel, width = int(channel_text), float(width_text)
                    if not 0 <= width <= 1:
                        raise ValueError
                except ValueError:
                    print("输入无效：请输入通道号和 0 到 1 的开度。")
                    continue
                gripper.goto(width, channel)
                result = gripper.wait(channel=channel)
                print(f"{channel} 号结果：{result['state']}。{result['message']}")
                print(f"{channel} 号当前开度：{gripper.get_width(channel):.3f}")
        finally:
            gripper.disconnect()
    except GripperError as exc:
        print(f"操作失败：{exc}")


if __name__ == "__main__":
    main()
