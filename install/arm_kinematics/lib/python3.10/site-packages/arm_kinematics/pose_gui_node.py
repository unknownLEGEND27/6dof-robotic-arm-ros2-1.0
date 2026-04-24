import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import tkinter as tk
import threading
import math
from tf_transformations import quaternion_from_euler


class PoseGUI(Node):

    def __init__(self):

        super().__init__('pose_gui_node')

        self.publisher = self.create_publisher(Pose, '/target_pose', 10)

        # 🔥 Root window
        self.root = tk.Tk()
        self.root.title("6DOF Arm Controller")
        self.root.geometry("500x500")
        self.root.configure(bg="#1e1e1e")

        # 🔥 Styles
        self.slider_style = {
            "from_": -1, "to": 1, "resolution": 0.01,
            "orient": tk.HORIZONTAL, "length": 350,
            "bg": "#1e1e1e", "fg": "white",
            "highlightthickness": 0
        }

        self.label_style = {
            "bg": "#1e1e1e",
            "fg": "#00ffcc",
            "font": ("Arial", 10, "bold")
        }

        # 🔥 Position Frame
        pos_frame = tk.LabelFrame(self.root, text="Position", fg="white", bg="#1e1e1e")
        pos_frame.pack(pady=10)

        self.x = self.create_slider(pos_frame, "X", -1, 1)
        self.y = self.create_slider(pos_frame, "Y", -1, 1)
        self.z = self.create_slider(pos_frame, "Z", 0, 1)

        # 🔥 Orientation Frame
        ori_frame = tk.LabelFrame(self.root, text="Orientation (RPY)", fg="white", bg="#1e1e1e")
        ori_frame.pack(pady=10)

        self.roll = self.create_slider(ori_frame, "Roll", -math.pi, math.pi)
        self.pitch = self.create_slider(ori_frame, "Pitch", -math.pi, math.pi)
        self.yaw = self.create_slider(ori_frame, "Yaw", -math.pi, math.pi)

        # 🔥 Buttons
        btn_frame = tk.Frame(self.root, bg="#1e1e1e")
        btn_frame.pack(pady=10)

        tk.Button(btn_frame, text="Send Pose", command=self.send_pose,
                  bg="#00ffcc", fg="black", width=15).pack(side=tk.LEFT, padx=5)

        tk.Button(btn_frame, text="Home", command=self.go_home,
                  bg="#ffaa00", fg="black", width=15).pack(side=tk.LEFT, padx=5)

        # 🔥 Auto send toggle
        self.auto_send = tk.BooleanVar()
        tk.Checkbutton(self.root, text="Auto Send",
                       variable=self.auto_send,
                       bg="#1e1e1e", fg="white",
                       selectcolor="#333").pack()

        # ROS spin thread
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        # Auto update loop
        self.update_loop()

    # 🔥 Slider creator
    def create_slider(self, parent, label, min_val, max_val):
        frame = tk.Frame(parent, bg="#1e1e1e")
        frame.pack(pady=5)

        tk.Label(frame, text=label, **self.label_style).pack(anchor="w")

        slider = tk.Scale(frame,
                          from_=min_val,
                          to=max_val,
                          resolution=0.01,
                          orient=tk.HORIZONTAL,
                          length=300,
                          bg="#1e1e1e",
                          fg="white",
                          highlightthickness=0)

        slider.pack()
        return slider

    # 🔥 Send Pose
    def send_pose(self):

        msg = Pose()

        x = self.x.get()
        y = self.y.get()
        z = self.z.get()

        roll = self.roll.get()
        pitch = self.pitch.get()
        yaw = self.yaw.get()

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.publisher.publish(msg)

        self.get_logger().info(
            f"x={x:.2f}, y={y:.2f}, z={z:.2f} | "
            f"r={roll:.2f}, p={pitch:.2f}, y={yaw:.2f}"
        )

    # 🔥 Home
    def go_home(self):

        self.x.set(0.3)
        self.y.set(0.0)
        self.z.set(0.4)

        self.roll.set(0.0)
        self.pitch.set(0.0)
        self.yaw.set(0.0)

        self.send_pose()

    # 🔥 Auto update loop
    def update_loop(self):
        if self.auto_send.get():
            self.send_pose()

        self.root.after(100, self.update_loop)


def main(args=None):

    rclpy.init(args=args)

    node = PoseGUI()

    node.root.mainloop()

    rclpy.shutdown()