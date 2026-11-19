FROM ros:humble-ros-base-jammy@sha256:4dd8083b7009624fd5e4b3c163f280bc42d29250b9d60e3126b49ce345d948c4

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid 10001 participant \
    && useradd --uid 10001 --gid 10001 --create-home --shell /bin/sh participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY --chmod=0755 entrypoint.sh /entrypoint.sh

USER participant
WORKDIR /home/participant
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
