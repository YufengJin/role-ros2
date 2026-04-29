# Dockerfile for the FACTR desktop-side teleoperation environment.
#
# This extends the existing ROS2 Humble/CUDA/ZED desktop image instead of
# adding FACTR-only dependencies to the shared ros2_cu118 image.

FROM ros2_cu118:latest

SHELL ["/bin/bash", "-c"]

ENV FACTR_CONFIG=/app/ros2_ws/src/role-ros2/config/factr_teleop_config.yaml \
    FACTR_FTDI_LATENCY_TIMER=1

# Utilities for USB serial inspection and latency tuning.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-tk \
    python3-serial \
    python3-yaml \
    setserial \
    udev \
    && rm -rf /var/lib/apt/lists/*

# FACTR runtime dependencies:
# - dynamixel-sdk: direct Dynamixel access for the leader arm
# - pin: Pinocchio Python bindings, imported as `pinocchio`
# - pyserial: serial backend used by the Dynamixel stack and useful for debug
RUN python3 -m pip install --no-cache-dir \
    dynamixel-sdk \
    pyserial \
    pin==3.4.0

RUN python3 -c "import dynamixel_sdk; import pinocchio as pin; print(f'FACTR deps OK: pinocchio {pin.__version__}')"

COPY scripts/factr_set_latency_timer.sh /usr/local/bin/factr-set-latency
RUN chmod +x /usr/local/bin/factr-set-latency && \
    echo "export PS1='[ros2_factr]\\u@\\h:\\w\\$ '" >> /root/.bashrc && \
    echo "export FACTR_CONFIG=${FACTR_CONFIG}" >> /root/.bashrc && \
    echo "export FACTR_FTDI_LATENCY_TIMER=${FACTR_FTDI_LATENCY_TIMER}" >> /root/.bashrc

WORKDIR /app/ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
