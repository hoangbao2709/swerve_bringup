#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotTeleopNode(Node):
    def __init__(self):
        super().__init__("swerve_gui_teleop")

        self.declare_parameter("speed", 0.30)
        self.declare_parameter("publish_rate", 10.0)

        self.speed = float(self.get_parameter("speed").value)
        rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self._lock = threading.Lock()
        self._x_dir = 0.0
        self._y_dir = 0.0

        self.timer = self.create_timer(1.0 / rate, self._publish_cmd)

    def set_speed(self, speed: float):
        with self._lock:
            self.speed = max(0.0, float(speed))

    def set_direction(self, x_dir: float, y_dir: float):
        with self._lock:
            self._x_dir = x_dir
            self._y_dir = y_dir

    def stop(self):
        with self._lock:
            self._x_dir = 0.0
            self._y_dir = 0.0

        self.publisher.publish(Twist())

    def _publish_cmd(self):
        with self._lock:
            vx = self._x_dir * self.speed
            vy = self._y_dir * self.speed

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = 0.0
        self.publisher.publish(msg)


class TeleopGUI:
    def __init__(self, node: RobotTeleopNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title("Swerve Robot Teleop")
        self.root.geometry("430x500")
        self.root.minsize(360, 440)

        # Root layout
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        title_frame = ttk.Frame(self.root, padding=(14, 12, 14, 4))
        title_frame.grid(row=0, column=0, sticky="ew")
        title_frame.columnconfigure(0, weight=1)

        ttk.Label(
            title_frame,
            text="SWERVE ROBOT CONTROL",
            font=("Arial", 18, "bold"),
            anchor="center",
        ).grid(row=0, column=0, sticky="ew")

        ttk.Label(
            title_frame,
            text="8 hướng di chuyển + STOP",
            font=("Arial", 10),
            anchor="center",
        ).grid(row=1, column=0, pady=(4, 0), sticky="ew")

        # Speed row
        speed_frame = ttk.Frame(self.root, padding=(16, 6, 16, 10))
        speed_frame.grid(row=1, column=0, sticky="ew")
        speed_frame.columnconfigure(1, weight=1)

        ttk.Label(speed_frame, text="Speed (m/s):").grid(
            row=0, column=0, padx=(0, 8)
        )

        self.speed_var = tk.DoubleVar(value=self.node.speed)

        self.speed_scale = ttk.Scale(
            speed_frame,
            from_=0.05,
            to=1.00,
            variable=self.speed_var,
            command=self._speed_changed,
        )
        self.speed_scale.grid(row=0, column=1, sticky="ew")

        self.speed_label = ttk.Label(
            speed_frame,
            text=f"{self.speed_var.get():.2f}",
            width=5,
            anchor="e",
        )
        self.speed_label.grid(row=0, column=2, padx=(8, 0))

        # 3x3 movement pad
        pad = ttk.Frame(self.root, padding=(14, 8, 14, 8))
        pad.grid(row=2, column=0, sticky="nsew")

        for r in range(3):
            pad.rowconfigure(r, weight=1, uniform="padrow")
        for c in range(3):
            pad.columnconfigure(c, weight=1, uniform="padcol")

        buttons = [
            ("↖", +1, +1), ("↑", +1,  0), ("↗", +1, -1),
            ("←",  0, +1), ("STOP", 0,  0), ("→",  0, -1),
            ("↙", -1, +1), ("↓", -1,  0), ("↘", -1, -1),
        ]

        for index, (text, x_dir, y_dir) in enumerate(buttons):
            row = index // 3
            col = index % 3

            if text == "STOP":
                btn = tk.Button(
                    pad,
                    text="STOP",
                    font=("Arial", 14, "bold"),
                    bg="#c81919",
                    fg="white",
                    activebackground="#e02a2a",
                    activeforeground="white",
                    relief="raised",
                    bd=2,
                    command=self._stop,
                )
            else:
                btn = tk.Button(
                    pad,
                    text=text,
                    font=("Arial", 24, "bold"),
                    relief="raised",
                    bd=2,
                    command=lambda x=x_dir, y=y_dir, t=text: self._move(x, y, t),
                )

            btn.grid(
                row=row,
                column=col,
                sticky="nsew",
                padx=5,
                pady=5,
                ipadx=2,
                ipady=2,
            )

        # Footer
        footer = ttk.Frame(self.root, padding=(14, 4, 14, 12))
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)

        self.status_var = tk.StringVar(value="STOPPED · /cmd_vel")

        ttk.Label(
            footer,
            textvariable=self.status_var,
            font=("Arial", 10, "bold"),
            anchor="center",
        ).grid(row=0, column=0, sticky="ew")

        ttk.Label(
            footer,
            text="W/A/S/D: di chuyển · Space: STOP",
            font=("Arial", 9),
            anchor="center",
        ).grid(row=1, column=0, pady=(4, 0), sticky="ew")

        # Keyboard shortcuts
        self.root.bind("<KeyPress-w>", lambda e: self._move(+1, 0, "↑"))
        self.root.bind("<KeyPress-s>", lambda e: self._move(-1, 0, "↓"))
        self.root.bind("<KeyPress-a>", lambda e: self._move(0, +1, "←"))
        self.root.bind("<KeyPress-d>", lambda e: self._move(0, -1, "→"))
        self.root.bind("<KeyPress-space>", lambda e: self._stop())

        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _move(self, x_dir: float, y_dir: float, label: str):
        self.node.set_direction(x_dir, y_dir)
        self.status_var.set(
            f"MOVING {label} · {self.speed_var.get():.2f} m/s · /cmd_vel"
        )

    def _stop(self):
        self.node.stop()
        self.status_var.set("STOPPED · /cmd_vel")

    def _speed_changed(self, _value=None):
        value = float(self.speed_var.get())
        self.node.set_speed(value)
        self.speed_label.config(text=f"{value:.2f}")

    def _close(self):
        self.node.stop()
        self.root.after(100, self.root.destroy)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = RobotTeleopNode()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    ros_thread.start()

    try:
        TeleopGUI(node).run()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
