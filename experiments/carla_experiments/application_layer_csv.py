import numpy as np
import rclpy
from rclpy.node import Node

from object_model_msgs.msg import ObjectModel, Object, Track, Dimensions
from fusion_layer.srv import RegisterSensor, RemoveSensor


class ApplicationLayerCsv(Node):

    def __init__(self):
        super().__init__('application_layer_csv')

        self._csv = open('application_layer.csv', 'w')
        self._counter = 1

        self.subscription = self.create_subscription(
            ObjectModel,
            'objectlevel_fusion/fusion_layer/fusion/get',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        id_name = msg.header.frame_id
        num_objs = len(msg.object_model)

        self.get_logger().info('Received %d objects from %s' % (num_objs, id_name))

        time_ = self.get_clock().now().from_msg(msg.header.stamp)
        timestamp = time_.nanoseconds

        for obj in msg.object_model:
            list_ = [timestamp, self._counter]
            list_.extend(list(np.round(obj.track.state, 5)))
            list_.extend(list(np.round(obj.dimensions.values, 5)))

            self._csv.write(','.join(map(str, list_)) + '\n')

        self._csv.flush()
        self._counter += 1

    def finish(self):
        self._csv.close()


def main(args=None):
    rclpy.init(args=args)

    application_layer_csv = ApplicationLayerCsv()

    rclpy.spin(application_layer_csv)

    application_layer_csv.destroy_node()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Finishing...')

        rclpy.shutdown()

        print('Finished!')
