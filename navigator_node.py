import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ObstacleAvoidanceNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')
        self.pub_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # --- DI SINI TEMPAT SETTING KECEPATAN JELAJAH & BELOKNYA ---
        self.BASE_THRUST = 140.0   # Kecepatan jelajah dasar (kisaran 120.0 - 150.0 N)
        self.TURN_GAIN = 45.0      # Selisih thruster untuk belok (kisaran 40.0 - 60.0 N)

        # Timer 10 Hz untuk mengirim perintah kontinu ke thruster WAM-V
        self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # Contoh logika sederhana berdasarkan hasil deteksi YOLOv8:
        # error_x bernilai dari -1.0 (obstacle di kiri) sampai +1.0 (obstacle di kanan)
        error_x = 0.0  # ganti dengan kalkulasi koordinat cx bounding box YOLOv8

        left_thrust = self.BASE_THRUST + (error_x * self.TURN_GAIN)
        right_thrust = self.BASE_THRUST - (error_x * self.TURN_GAIN)

        self.pub_left.publish(Float64(data=float(left_thrust)))
        self.pub_right.publish(Float64(data=float(right_thrust)))