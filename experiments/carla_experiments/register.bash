#!/bin/bash

ros2 service call /fusion_layer/register_sensor fusion_layer/srv/RegisterSensor "name: 'sensor1'
x: 1.0
y: -2.0
angle: 0.7853981633974483
capable:
- true
- true
- true
- true
- true
- true
- true
- true
measurement_noise_matrix:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
"

ros2 service call /fusion_layer/register_sensor fusion_layer/srv/RegisterSensor "name: 'sensor2'
x: -1.0
y: 0.5
angle: -1.0471975511965976
capable:
- true
- true
- true
- true
- true
- true
- true
- true
measurement_noise_matrix:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
"
