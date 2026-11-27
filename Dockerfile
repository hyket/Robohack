# syntax=docker/dockerfile:1
ARG ROS_IMAGE=ros:humble-ros-base-jammy@sha256:4dd8083b7009624fd5e4b3c163f280bc42d29250b9d60e3126b49ce345d948c4

FROM ${ROS_IMAGE} AS unitree_interfaces
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake python3-colcon-common-extensions \
        ros-humble-geometry-msgs ros-humble-rosidl-default-generators \
        ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /build
COPY T2/t2_checked/vendor/unitree_go ./src/unitree_go
RUN source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential \
        --install-base /opt/unitree --packages-select unitree_go \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ${ROS_IMAGE} AS participant_base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libglfw3 python3-numpy python3-pip python3-zipp \
        ros-humble-geometry-msgs ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid 10001 participant \
    && useradd --uid 10001 --gid 10001 --create-home --shell /bin/sh participant

COPY T2/t2_checked/requirements.lock /opt/t2/requirements.lock
RUN python3 -m pip install --no-cache-dir --no-deps --only-binary=:all: \
    --require-hashes -r /opt/t2/requirements.lock
COPY --from=unitree_interfaces /opt/unitree /opt/unitree
COPY T2/t2_checked/model/ /opt/t2/model/
COPY T2/t2_checked/safety_block.py T2/t2_checked/collision_guard.py \
    T2/t2_checked/t2_collision_pairs.csv /opt/t2/
COPY T2/t2_checked/NOTICE-go1-MIT.txt T2/t2_checked/vendor/NOTICE-unitree-BSD.txt /usr/local/share/doc/t2/

FROM participant_base
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONPATH=/opt/t2 \
    PYTHONUNBUFFERED=1 \
    OPENBLAS_NUM_THREADS=1 \
    OMP_NUM_THREADS=1
COPY T3/policy.py T3/controller.py T3/participant.py /opt/t3/
COPY T3/checkpoints/walk_these_ways.npz T3/checkpoints/LICENSE-walk-these-ways.txt \
    T3/checkpoints/README.md /opt/t3/checkpoints/
COPY --chmod=0755 entrypoint.sh /entrypoint.sh

USER participant
WORKDIR /opt/t3
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
