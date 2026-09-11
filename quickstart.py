"""输入 0 到 1 的开度，演示夹爪控制与当前开度读取。"""

from gripper import Gripper, GripperError


def main():
    try:
        with Gripper() as gripper:
            if not gripper.home():
                return
            print(f"回零后的当前开度：{gripper.get_width():.3f}")
            choice = input("输入 0 到 1 的目标开度，或 q 退出：").strip().lower()
            if choice == "q":
                return
            try:
                target_width = float(choice)
            except ValueError:
                print("输入无效：请输入 0 到 1 之间的数字，或 q。")
                return
            if not 0 <= target_width <= 1:
                print("输入无效：目标开度必须在 0 到 1 之间。")
                return
            gripper.goto(target_width)
            print("目标已更新，后台正在持续检查位置、负载和电流……")
            result = gripper.wait()
            print(f"结果：{result['state']}。{result['message']}")
            print(f"当前开度：{gripper.get_width():.3f}")
    except GripperError as exc:
        print(f"操作失败：{exc}")


if __name__ == "__main__":
    main()
