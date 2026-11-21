# syntax=docker/dockerfile:1
ARG ROS_IMAGE=ros:humble-ros-base-jammy

FROM ${ROS_IMAGE} AS unitree_interfaces
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake python3-colcon-common-extensions \
        ros-humble-geometry-msgs ros-humble-rosidl-default-generators \
        ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /build
COPY vendor/unitree_go ./src/unitree_go
RUN source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential \
        --install-base /opt/unitree --packages-select unitree_go \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${ROS_IMAGE} AS participant
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libglfw3 python3-numpy python3-pip python3-zipp \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.lock /opt/t2/requirements.lock
RUN python3 -m pip install --no-cache-dir --no-deps --only-binary=:all: \
    --require-hashes -r /opt/t2/requirements.lock
COPY --from=unitree_interfaces /opt/unitree /opt/unitree
COPY model/ /opt/t2/model/
COPY safety_block.py collision_guard.py t2_checks.py t2_collision_pairs.csv /opt/t2/
COPY NOTICE-go1-MIT.txt /usr/local/share/doc/t2/
COPY vendor/NOTICE-unitree-BSD.txt /usr/local/share/doc/t2/
COPY --chmod=0755 entrypoint.sh /entrypoint.sh
RUN useradd --create-home --uid 10001 participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1
USER participant
WORKDIR /opt/t2
RUN --network=none source /opt/ros/humble/setup.bash \
    && source /opt/unitree/local_setup.bash \
    && python3 -c 'from rclpy.type_support import check_for_type_support; from unitree_go.msg import LowCmd; check_for_type_support(LowCmd); assert len(LowCmd().motor_cmd) == 20' \
    && python3 /opt/t2/safety_block.py --self-test \
    && python3 /opt/t2/safety_block.py --collision-test
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
