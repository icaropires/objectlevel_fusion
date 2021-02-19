# Meant to be built having the repository root as context
FROM ros:foxy-ros-base

RUN mkdir -p /ros2_ws/src

COPY . /ros2_ws/src

WORKDIR /ros2_ws

RUN apt update -y \
  && rosdep install -i --from-path src --rosdistro foxy -y \
  && . /opt/ros/foxy/setup.sh \
  && colcon build --event-handlers console_cohesion+ \
  && ln -s /ros2_ws/src/docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["/opt/ros/foxy/bin/ros2", "run", "fusion_layer", "fusion"]
