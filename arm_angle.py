import leap
import time
import socket
from timeit import default_timer as timer
from typing import Callable
from leap.events import TrackingEvent
from leap.event_listener import LatestEventListener
from leap.datatypes import FrameData
import tkinter as tk
from scipy.spatial.transform import Rotation as R

def wait_until(condition: Callable[[], bool], timeout: float = 5, poll_delay: float = 0.01):
    """
    等待直到指定条件为真，或者超时。可以指定超时时间和轮询间隔。
    """
    start_time = timer()
    while timer() - start_time < timeout:
        if condition():
            return True
        time.sleep(poll_delay)
    if not condition():
        return False

def update_gui_position(x, y, z, roll, pitch, yaw):
    """
    更新GUI中的坐标显示，精确到小数点后一位，并显示欧拉角
    """
    x_label.config(text=f"X: {x:.1f}")
    y_label.config(text=f"Y: {y:.1f}")
    z_label.config(text=f"Z: {z:.1f}")
    roll_label.config(text=f"Roll: {roll:.1f}")
    pitch_label.config(text=f"Pitch: {pitch:.1f}")
    yaw_label.config(text=f"Yaw: {yaw:.1f}")

def main():
    # 设置UDP目标地址和端口
    udp_ip = "127.0.0.1"  # 目标IP地址
    udp_port = 5005       # 目标端口
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 创建 Leap API 的事件监听器和连接
    tracking_listening = LatestEventListener(leap.EventType.Tracking)
    connection = leap.Connection()
    connection.add_listener(tracking_listening)

    # 使用上下文管理器打开 Leap 连接
    with connection.open() as open_connection:
        # 等待直到 Tracking 事件不为空
        wait_until(lambda: tracking_listening.event is not None)

        # 主循环，用于更新右手位置并通过UDP发送数据
        while True:
            event = tracking_listening.event
            if event is None:
                continue
            event_timestamp = event.timestamp

            # 初始化目标帧大小和帧时间的存储指针
            target_frame_size = leap.ffi.new("uint64_t*")
            frame_time = leap.ffi.new("int64_t*")
            frame_time[0] = event_timestamp

            # 模拟 20 毫秒延迟
            time.sleep(0.02)

            try:
                leap.get_frame_size(open_connection, frame_time, target_frame_size)
            except Exception as e:
                print("get_frame_size() failed with: ", e)
                continue

            frame_data = FrameData(target_frame_size[0])
            try:
                leap.interpolate_frame(
                    open_connection,
                    event_timestamp + 30000,
                    frame_data.frame_ptr(),
                    target_frame_size[0],
                )
            except Exception as e:
                print("interpolate_frame() failed with: ", e)
                continue

            event = TrackingEvent(frame_data)

            for hand in event.hands:
                if str(hand.type) == "HandType.Right":
                    hand_position = (
                        hand.palm.position.x,
                        hand.palm.position.y,
                        hand.palm.position.z
                    )

                    # 获取四元数并计算欧拉角
                    quaternion = [
                        hand.palm.orientation[0],
                        hand.palm.orientation[1],
                        hand.palm.orientation[2],
                        hand.palm.orientation[3]
                    ]
                    rotation = R.from_quat(quaternion)
                    euler_angles = rotation.as_euler('zxy', degrees=True)
                    euler_angles2 = rotation.as_euler('xyz', degrees=True)
                    
                    # 更新GUI显示
                    update_gui_position(
                        hand_position[0], hand_position[1], hand_position[2],
                        euler_angles[0], euler_angles[1], euler_angles[2]
                    )

                    # 将右手位置和欧拉角数据通过UDP发送
                    message = f"{hand_position[0]:.1f},{hand_position[1]:.1f},{hand_position[2]:.1f}," \
                              f"{euler_angles[0]:.1f},{euler_angles[1]:.1f},{euler_angles[2]:.1f}," \
                              f"{euler_angles2[0]:.1f},{euler_angles2[1]:.1f},{euler_angles2[2]:.1f}"
                    udp_socket.sendto(message.encode(), (udp_ip, udp_port))

            root.update_idletasks()  # 更新GUI界面

# GUI 初始化
root = tk.Tk()
root.title("Right Hand Position and Orientation")

# 坐标显示标签
x_label = tk.Label(root, text="X: 0.0", font=("Helvetica", 14))
x_label.pack()
y_label = tk.Label(root, text="Y: 0.0", font=("Helvetica", 14))
y_label.pack()
z_label = tk.Label(root, text="Z: 0.0", font=("Helvetica", 14))
z_label.pack()

# 欧拉角显示标签
roll_label = tk.Label(root, text="Roll: 0.0", font=("Helvetica", 14))
roll_label.pack()
pitch_label = tk.Label(root, text="Pitch: 0.0", font=("Helvetica", 14))
pitch_label.pack()
yaw_label = tk.Label(root, text="Yaw: 0.0", font=("Helvetica", 14))
yaw_label.pack()

# 启动主函数
if __name__ == "__main__":
    main()
    root.mainloop()
