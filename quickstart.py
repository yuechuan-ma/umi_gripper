"""演示单通道或双通道夹爪控制。"""

from gripper import Gripper, GripperError


def main():
    try:
        with Gripper() as gripper:
            channels = sorted(gripper.channels)
            for channel in channels:
                if not gripper.home(channel):
                    return
                print(f"{channel} 号回零后的当前开度：{gripper.get_width(channel):.3f}")
            while True:
                choice = (
                    input("输入“通道号 开度”（例如 0 0.8），或 q 退出：")
                    .strip()
                    .lower()
                )
                if choice == "q":
                    return
                try:
                    channel_text, width_text = choice.split()
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
    except GripperError as exc:
        print(f"操作失败：{exc}")


if __name__ == "__main__":
    main()
