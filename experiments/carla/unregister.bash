#!/bin/bash
ros2 service call /fusion_layer/remove_sensor fusion_layer/srv/RemoveSensor "{name: sensor1}"
ros2 service call /fusion_layer/remove_sensor fusion_layer/srv/RemoveSensor "{name: sensor2}"
