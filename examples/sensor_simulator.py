import numpy as np
import rclpy
from rclpy.node import Node

from object_model_msgs.msg import ObjectModel, Track
from fusion_layer.srv import RegisterSensor, RemoveSensor


class CameraSensor(Node):

    def __init__(self, name, x, y, angle, capable, measurement_noise_matrix):
        super().__init__(name)

        self.publisher_ = self.create_publisher(ObjectModel, 'objectlevel_fusion/fusion_layer/fusion/submit', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.x = x
        self.y = y
        self.angle = angle
        self.capable = capable
        self.measurement_noise_matrix = measurement_noise_matrix

        # TODO: Make it a list of objects
        self.obj1_measurements = ObjectMeasurements(self.get_clock())

        self.sensor_registration_client = self.create_client(RegisterSensor, 'fusion_layer/register_sensor')
        while not self.sensor_registration_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Failed to connect for sensor registration, trying again...')

        self.sensor_remover_client = self.create_client(RemoveSensor, 'fusion_layer/remove_sensor')
        while not self.sensor_remover_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Failed to connect for sensor removing, trying again...')

        # self._unregister()  # Facilitate running many small tests in a row
        self._register()

    def _register(self):
        request = RegisterSensor.Request()

        request.name = self.get_name()
        request.x = self.x
        request.y = self.y
        request.angle = self.angle
        request.capable = self.capable
        request.measurement_noise_matrix = self.measurement_noise_matrix.reshape(-1)

        self.sensor_registration_client.call_async(request)
        self.get_logger().info(f"Sensor {self.get_name()} registered successfully!")

    def _unregister(self):
        request = RemoveSensor.Request()
        request.name = self.get_name()

        return self.sensor_remover_client.call_async(request)

    def timer_callback(self):
        msg = ObjectModel()
        msg.header.frame_id = self.get_name()

        msg.track.state, time_stamp = next(self.obj1_measurements)
        msg.header.stamp = time_stamp.to_msg()

        self.publisher_.publish(msg)

        state_truncated = list(np.round(msg.track.state, 4))
        self.get_logger().info('Publishing state: %s' % state_truncated)


class ObjectMeasurements:

    def __init__(self, clock, pos=0., velocity=0., acceleration=0.5, yaw=0., yaw_rate=0.):
        self.clock = clock
        self.time_last_msg = self.clock.now()

        self.pos = pos
        self.velocity = velocity
        self.acceleration = velocity
        self.yaw = yaw
        self.yaw_rate = yaw_rate

        self.acc_function = self._acc_increasing
        self.random_generator = np.random.default_rng()

    def __next__(self):
        """ Generates a simulated measurement from this object
        Increases/decreases yaw_rate and acceleration randomly and the other state attributes
        are calculated according.
        """
        now = self.clock.now()

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

        self.yaw += delta_t * self.yaw_rate
        self.yaw = self.yaw % (2 * np.pi)
        self.velocity += delta_t * self.acceleration
        self.pos += delta_t * self.velocity

        state = np.zeros(Track.STATE_SIZE, dtype='float32')

        state[Track.STATE_X_IDX] = self.pos * np.cos(self.yaw)
        state[Track.STATE_Y_IDX] = self.pos * np.sin(self.yaw)
        state[Track.STATE_VELOCITY_X_IDX] = self.velocity * np.cos(self.yaw)
        state[Track.STATE_VELOCITY_Y_IDX] = self.velocity * np.sin(self.yaw)
        state[Track.STATE_ACCELERATION_X_IDX] = self.acceleration * np.cos(self.yaw)
        state[Track.STATE_ACCELERATION_Y_IDX] = self.acceleration * np.sin(self.yaw)
        state[Track.STATE_YAW_IDX] = self.yaw
        state[Track.STATE_YAW_RATE_IDX] = self.yaw_rate

        state = self._add_noise(state)

        return state, now

    def _get_delta_t(self, time):
        delta_t_seconds = time.nanoseconds - self.time_last_msg.nanoseconds

        return delta_t_seconds / 10**9

    def _add_noise(self, state):
        # Check experiments/temporal_alignment_EKF.ipynb
        sensors_std = [1.5, 1.5, 0.9998, 0.02, 0.4999, 0.01, 0.02, 0.1]

        # taken from experiments/temporal_alignment_EKF.ipynb
        noises = (self.random_generator.random() for _ in sensors_std)
        noises = np.array([n * 2*std - std for std, n in zip(sensors_std, noises)])

        state += noises

        return state

    def _acc_increasing(self):
        return self.random_generator.random() * 0.05 - 0.02

    def _acc_decreasing(self):
        return self.random_generator.random() * 0.05 - 0.03


def main(args=None):
    rclpy.init(args=args)

    capable = [True] * Track.STATE_SIZE
    measurement_noise_matrix = np.diag([1.5**2, 1.5**2, 1**2, 0.5**2, 0.02**2, 0.1**2]).astype('float32')

    camera1 = CameraSensor('camera1', 5., -2., np.pi/4, capable, measurement_noise_matrix)

    rclpy.spin(camera1)


if __name__ == '__main__':
    main()
