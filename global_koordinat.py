#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import tf2_ros


class KinematicModel(Node):
    """
    Menerapkan model kinematika robot (unicycle) dengan langkah waktu TETAP dt = 0.01 s:

        x_dot     = V_B * cos(theta)
        y_dot     = V_B * sin(theta)
        theta_dot = omega

    Tiap langkah (0.01 detik), model diperbarui dengan metode Euler:

        x     += V_B * cos(theta) * dt
        y     += V_B * sin(theta) * dt
        theta += omega * dt

    Cara kerja:
      - Kamu ketik V_B, omega (dan durasi) di terminal -> dikirim ke /cmd_vel
      - Pose PREDIKSI dihitung dari model di atas
      - Pose NYATA dibaca dari TF (odom -> base_footprint)
      - Keduanya dibandingkan saat robot berhenti
    """

    DT = 0.01  # langkah waktu tetap (detik)

    def __init__(self):
        super().__init__('kinematic_model')

        self.dt = self.DT

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Pose nyata dari TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Perintah aktif
        self.v = 0.0
        self.w = 0.0
        self.active = False
        self.steps_left = None   # None = jalan terus sampai 's'
        self.step_count = 0

        # Pose model (diisi dari pose nyata saat TF pertama kali tersedia)
        self.mx = None
        self.my = None
        self.mth = None

        # Timer: loop dipanggil setiap dt detik
        self.create_timer(self.dt, self.loop)

        threading.Thread(target=self.input_loop, daemon=True).start()

    # ---------- Pose nyata dari TF ----------
    def get_real_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None

        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ---------- Loop utama: dipanggil tiap 0.01 detik ----------
    def loop(self):
        real = self.get_real_pose()

        # Sinkronkan pose model dengan pose nyata saat pertama kali
        if self.mx is None:
            if real is None:
                self.get_logger().warn(
                    'Menunggu TF odom -> base_footprint ...',
                    throttle_duration_sec=2.0)
                return
            self.mx, self.my, self.mth = real
            print(f'\nTF tersedia. Pose awal: x={real[0]:.3f}, '
                  f'y={real[1]:.3f}, theta={math.degrees(real[2]):.1f} deg')
            print('v w> ', end='', flush=True)

        if not self.active:
            return

        # Durasi habis -> berhenti
        if self.steps_left is not None and self.steps_left <= 0:
            self.stop()
            return

        # Kirim perintah ke robot
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.cmd_pub.publish(msg)

        # Update model kinematika (Euler, dt tetap)
        self.mx += self.v * math.cos(self.mth) * self.dt
        self.my += self.v * math.sin(self.mth) * self.dt
        self.mth = self.wrap(self.mth + self.w * self.dt)

        self.step_count += 1
        if self.steps_left is not None:
            self.steps_left -= 1

        # Tampilkan status 1 kali per detik (tiap 100 langkah)
        if self.step_count % round(1.0 / self.dt) == 0:
            self.print_status(real)

    def print_status(self, real):
        line = (f'\n[model] x={self.mx:7.3f} y={self.my:7.3f} '
                f'th={math.degrees(self.mth):7.1f}')
        if real is not None:
            line += (f'  |  [nyata] x={real[0]:7.3f} y={real[1]:7.3f} '
                     f'th={math.degrees(real[2]):7.1f}')
        print(line)

    def stop(self):
        self.active = False
        self.steps_left = None
        self.cmd_pub.publish(Twist())

        real = self.get_real_pose()
        elapsed = self.step_count * self.dt
        print(f'\nRobot berhenti. ({self.step_count} langkah x {self.dt} s '
              f'= {elapsed:.2f} s)')
        print(f'  Model : x={self.mx:.3f} m, y={self.my:.3f} m, '
              f'theta={math.degrees(self.mth):.1f} deg')
        if real is not None:
            err_x = real[0] - self.mx
            err_y = real[1] - self.my
            err_th = math.degrees(self.wrap(real[2] - self.mth))
            print(f'  Nyata : x={real[0]:.3f} m, y={real[1]:.3f} m, '
                  f'theta={math.degrees(real[2]):.1f} deg')
            print(f'  Selisih (nyata - model): dx={err_x:.3f} m, '
                  f'dy={err_y:.3f} m, dtheta={err_th:.1f} deg')
        print('v w> ', end='', flush=True)

    # ---------- Input dari terminal ----------
    def input_loop(self):
        print(f'Langkah waktu tetap dt = {self.DT} s')
        print('Format: v w        -> jalan terus sampai ketik s')
        print('        v w t      -> jalan selama t detik lalu berhenti')
        print('        s = berhenti, r = samakan pose model dengan pose nyata')
        print('Contoh: 0.2 0.0 5   (maju 0.2 m/s selama 5 detik)')
        while rclpy.ok():
            try:
                text = input('v w> ').strip().lower()
            except EOFError:
                return

            if text == 's':
                self.stop()
                continue

            if text == 'r':
                real = self.get_real_pose()
                if real is None:
                    print('TF belum tersedia.')
                else:
                    self.mx, self.my, self.mth = real
                    print('Pose model disamakan dengan pose nyata.')
                continue

            try:
                nums = list(map(float, text.split()))
                if len(nums) not in (2, 3):
                    raise ValueError
            except ValueError:
                print('Format salah. Contoh: 0.2 0.0 5')
                continue

            if self.mx is None:
                print('Belum ada TF, tunggu sebentar.')
                continue

            self.v, self.w = nums[0], nums[1]
            self.step_count = 0
            if len(nums) == 3:
                self.steps_left = int(round(nums[2] / self.dt))
            else:
                self.steps_left = None
            self.active = True
            print(f'Mulai: V_B={self.v} m/s, omega={self.w} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = KinematicModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()