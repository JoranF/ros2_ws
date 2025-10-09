#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smartcar_msgs.msg import Status  # of vervang door het juiste type

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_pub')

        # parameters
        self.declare_parameter('wheel_diameter', 0.064)
        self.declare_parameter('wheelbase_length', 0.257)
        self.declare_parameter('publish_rate', 50.0)

        self.wheel_radius = self.get_parameter('wheel_diameter').value / 2.0
        self.L = self.get_parameter('wheelbase_length').value
        self.rate = self.get_parameter('publish_rate').value

        # publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # subscribers
        self.sub = self.create_subscription(Status,
                                            '/smartcar/vehicle_status',
                                            self.status_cb, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)

        # state
        self.engine_rpm = 0.0
        self.steer_angle = 0.0
        self.wheel_pos = 0.0  # rad, integratie van wielrotatie
        self.last_time = self.get_clock().now()

    def status_cb(self, msg: Status):
        self.engine_rpm = float(msg.engine_speed_rpm)
        self.steer_angle = float(msg.steering_angle_rad)

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # bereken wielrotatie uit rpm
        rad_per_sec = (self.engine_rpm * 2.0 * math.pi) / 60.0
        self.wheel_pos += rad_per_sec * dt

        # JointState bericht
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            'back_left_wheel_joint',
            'back_right_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'front_left_wheel_steer_joint',
            'front_right_wheel_steer_joint'
        ]
        js.position = [
            self.wheel_pos,  # back left
            self.wheel_pos,  # back right
            self.wheel_pos,  # front left wheel draait
            self.wheel_pos,  # front right wheel draait
            self.steer_angle,  # stuurhoek links
            self.steer_angle   # stuurhoek rechts
        ]
        self.joint_pub.publish(js)

        # TF voorbeeld: stuurhoek links
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'front_left_mount'
        t.transform.translation.x = self.L/2.0
        t.transform.translation.y = self.L/2.0
        t.transform.translation.z = self.wheel_radius
        # Voorbeeld zonder rotatie → rviz2 ziet montagepositie
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()