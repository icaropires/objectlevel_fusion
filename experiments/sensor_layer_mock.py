import numpy as np
import rclpy
from rclpy.node import Node

from object_model_msgs.msg import ObjectModel, Track


class CameraSensor(Node):

    def __init__(self):
        super().__init__('camera1')
        self.publisher_ = self.create_publisher(ObjectModel, 'objectlevel_fusion/fusion_layer/fusion/submit', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.random_generator = np.random.default_rng()

        self.time_last_msg = self.get_clock().now().to_msg()
        self.pos = 0
        self.velocity = 0
        self.acceleration = 0.5
        self.yaw = 0
        self.yaw_rate = 0

        self.acc_function = self._acc_increasing

    def _get_delta_t(self, time):
        last = self.time_last_msg.sec * 10**9 + self.time_last_msg.nanosec
        time = time.sec * 10**9 + time.nanosec

        return (time - last) / 10**9

    def _add_noise(self, msg):
        sensors_std = [1.5, 1.5, 0.9998, 0.02, 0.4999, 0.01, 0.02, 0.1]

        # taken from experiments/temporal_alignment_EKF.ipynb
        noises = (self.random_generator.random() for _ in sensors_std)
        noises = np.array([n * 2*std - std for std, n in zip(sensors_std, noises)])

        msg.track.state += noises

    def _acc_increasing(self):
        return self.random_generator.random() * 0.05 - 0.02

    def _acc_decreasing(self):
        return self.random_generator.random() * 0.05 - 0.03

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        delta_t = self._get_delta_t(now)
        self.time_last_msg = now

        # Multiply by the desired interval length and sum the start of the interval
        self.yaw_rate += self.random_generator.random() * 0.03 - 0.015

        # Let's make acceleration more dynamic!
        if self.acceleration < 1:
            self.acc_function = self._acc_increasing
        elif self.acceleration > 3.5:
            self.acc_function = self._acc_decreasing

        self.acceleration += self.acc_function()
        print('Acc after', self.acceleration)

        self.yaw += delta_t * self.yaw_rate
        self.yaw = self.yaw % (2 * np.pi)
        self.velocity += delta_t * self.acceleration
        self.pos += delta_t * self.velocity

        msg = ObjectModel()
        msg.header.frame_id = self.get_name()
        msg.header.stamp = self.time_last_msg

        msg.track.state[Track.STATE_X_IDX] = self.pos * np.cos(self.yaw)
        msg.track.state[Track.STATE_Y_IDX] = self.pos * np.sin(self.yaw)
        msg.track.state[Track.STATE_VELOCITY_X_IDX] = self.velocity * np.cos(self.yaw)
        msg.track.state[Track.STATE_VELOCITY_Y_IDX] = self.velocity * np.sin(self.yaw)
        msg.track.state[Track.STATE_ACCELERATION_X_IDX] = self.acceleration * np.cos(self.yaw)
        msg.track.state[Track.STATE_ACCELERATION_Y_IDX] = self.acceleration * np.sin(self.yaw)
        msg.track.state[Track.STATE_YAW_IDX] = self.yaw
        msg.track.state[Track.STATE_YAW_RATE_IDX] = self.yaw_rate

        self._add_noise(msg)

        self.publisher_.publish(msg)
        state_truncated = list(np.round(msg.track.state, 4))
        self.get_logger().info('Publishing state: %s' % state_truncated)


def main(args=None):
    rclpy.init(args=args)

    camera_sensor = CameraSensor()

    rclpy.spin(camera_sensor)


if __name__ == '__main__':
    main()
