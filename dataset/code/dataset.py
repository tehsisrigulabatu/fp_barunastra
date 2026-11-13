import os, time, cv2, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OBSTACLE_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "obstacle"))
DEPAN_DIR = os.path.join(OBSTACLE_DIR, "cam_depan")
OBJEK_DIR = os.path.join(OBSTACLE_DIR, "cam_objek")

os.makedirs(DEPAN_DIR, exist_ok=True)
os.makedirs(OBJEK_DIR, exist_ok=True)

class DatasetHelper(Node):
    def __init__(self):
        super().__init__("dataset_helper")
        self.bridge = CvBridge()
        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd_time = 0.0
        self.last_save = 0.0
        self.count = 0

        self.pub_l = self.create_publisher(Float64, "/wamv/thrusters/left/thrust", 10)
        self.pub_r = self.create_publisher(Float64, "/wamv/thrusters/right/thrust", 10)

        self.create_subscription(Twist, "/cmd_vel", self.cb_vel, 10)
        self.create_subscription(Image, "/wamv/sensors/cameras/front_left_camera_sensor/image_raw", self.cb_depan, 10)
        self.create_subscription(Image, "/wamv/sensors/cameras/middle_right_camera_sensor/image_raw", self.cb_objek, 10)

        self.img_depan = None
        self.img_objek = None
        self.create_timer(0.1, self.loop)

    def cb_vel(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.last_cmd_time = time.time()

    def cb_depan(self, msg):
        self.img_depan = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cb_objek(self, msg):
        self.img_objek = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def loop(self):
        now = time.time()

        # AUTO-NETRAL: Jika tidak ada tombol baru ditekan selama 0.4 detik, otomatis rem/netral (0 N)
        if now - self.last_cmd_time > 0.4:
            self.linear = 0.0
            self.angular = 0.0

        # Belok halus (25 N saja biar tidak muter liar seperti gasing)
        base = self.linear * 150.0
        diff = self.angular * 25.0
        self.pub_l.publish(Float64(data=float(base - diff)))
        self.pub_r.publish(Float64(data=float(base + diff)))

        # Simpan foto tiap 0.5 detik
        if self.img_depan is not None and (now - self.last_save >= 0.5):
            cv2.imwrite(os.path.join(DEPAN_DIR, f"depan_{self.count:04d}.jpg"), self.img_depan)
            if self.img_objek is not None:
                cv2.imwrite(os.path.join(OBJEK_DIR, f"objek_{self.count:04d}.jpg"), self.img_objek)
            self.count += 1
            self.last_save = now
            print(f"[SAVED] Set ke-{self.count}")

rclpy.init()
rclpy.spin(DatasetHelper())