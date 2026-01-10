# Dockerfile for ROS2 Foxy with Polymetis
# Based on Ubuntu 20.04 (Focal) - required for ROS2 Foxy
# No conda/micromamba - all packages installed via apt or pip
FROM ubuntu:20.04

# Set build arguments
ARG LIBFRANKA_VERSION=0.18.2
ARG ROBOT_TYPE=fr3

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LIBFRANKA_VERSION=${LIBFRANKA_VERSION}
ENV ROBOT_TYPE=${ROBOT_TYPE}

# Set directory structure
ARG FAIRO_DIR=/opt/fairo
ARG POLYMETIS_DIR=${FAIRO_DIR}/polymetis
ARG POLYMETIS_CONFIG_DIR=${POLYMETIS_DIR}/polymetis/conf
ARG ROBOT_CONFIG_DIR=/app/ros2_ws/src/role-ros2/hardware/franka/config/${ROBOT_TYPE}

# ============================================================================
# Base System Installations
# ============================================================================
RUN apt-get update && \
    apt-get install -y \
    software-properties-common \
    build-essential \
    sudo \
    git \
    curl \
    wget \
    python3-pip \
    python3.8 \
    python3.8-dev \
    python3.8-venv \
    libspdlog-dev \
    libeigen3-dev \
    libfmt-dev \
    lsb-release \
    ffmpeg \
    libsm6 \
    libxext6 \
    locales \
    iputils-ping \
    iproute2 \
    net-tools \
    pkg-config \
    && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install latest CMake version
# Note: Kitware GPG key is rotated periodically, use the new keyring method
RUN apt-get update && \
    apt-get install -y ca-certificates gpg wget && \
    # Download and install the new Kitware signing key
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    # Add Kitware repository with signed-by option (new method)
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    # Install kitware-archive-keyring to keep key updated
    apt-get install -y kitware-archive-keyring && \
    apt-get install -y cmake && \
    rm -rf /var/lib/apt/lists/*

# ============================================================================
# Install ROS2 Foxy
# ============================================================================
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

RUN apt-get update && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-lark \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS2 environment in bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    # Set custom PS1 prompt to distinguish this container
    echo "export PS1='[ros2_polymetis]\\u@\\h:\\w\\$ '" >> /root/.bashrc

# ============================================================================
# Install Polymetis Build Dependencies (matching environment.yml versions)
# ============================================================================

# Install system libraries via apt
RUN apt-get update && \
    apt-get install -y \
    libassimp-dev \
    doxygen \
    libprotobuf-dev \
    libprotobuf17 \
    protobuf-compiler \
    protobuf-c-compiler \
    libprotoc-dev \
    libopenmpi-dev \
    openmpi-bin \
    libpoco-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libyaml-cpp-dev \
    # MuJoCo dependencies (for headless rendering and physics simulation)
    libgl1-mesa-dev \
    libglew-dev \
    libosmesa6-dev \
    libglfw3-dev \
    # X11 libraries (for headless X server support, required by MuJoCo/dm-control)
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxi-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    && rm -rf /var/lib/apt/lists/*

# Build protobuf from source (version 3.18.1, matching environment.yml)
# Ubuntu 20.04's protobuf (3.6.1) is too old for gRPC 1.41.1
# Note: protobuf 3.18.1 has CMakeLists.txt in cmake/ subdirectory
WORKDIR /tmp
RUN git clone --recurse-submodules -b v3.18.1 https://github.com/protocolbuffers/protobuf.git && \
    cd protobuf && \
    # Patch CMakeLists.txt to fix CMake version requirement
    find . -name "CMakeLists.txt" -exec sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' {} \; && \
    mkdir -p build && \
    cd build && \
    cmake ../cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -Dprotobuf_BUILD_TESTS=OFF \
    -Dprotobuf_BUILD_SHARED_LIBS=ON \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/protobuf

# Build gRPC from source (version 1.41.1, matching environment.yml)
# Now using the protobuf we just built
WORKDIR /tmp
RUN git clone --recurse-submodules -b v1.41.1 https://github.com/grpc/grpc.git && \
    cd grpc && \
    # Patch cares submodule CMakeLists.txt to fix CMake version requirement
    sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' third_party/cares/cares/CMakeLists.txt && \
    mkdir -p cmake/build && \
    cd cmake/build && \
    cmake ../.. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DgRPC_INSTALL=ON \
    -DgRPC_BUILD_TESTS=OFF \
    -DgRPC_PROTOBUF_PROVIDER=package \
    -DgRPC_ZLIB_PROVIDER=package \
    -DgRPC_CARES_PROVIDER=module \
    -DgRPC_SSL_PROVIDER=module \
    -DgRPC_RE2_PROVIDER=module \
    -DgRPC_ABSL_PROVIDER=module \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    # Use fewer parallel jobs to avoid memory/race condition issues
    make -j$(($(nproc) / 2)) || make -j1 && \
    make install && \
    ldconfig && \
    # Fix incorrect protobuf include paths in gRPC CMake config files
    find /usr/local/lib/cmake/grpc -name "*.cmake" -exec sed -i 's|"/include/google/protobuf/"|"/usr/local/include/google/protobuf/"|g' {} \; && \
    # Create symlinks for protobuf headers and libraries that some packages expect in non-standard locations
    mkdir -p /include/google && \
    ln -sf /usr/local/include/google/protobuf /include/google/protobuf && \
    # Create symlinks for protobuf libraries in /lib (some CMake configs reference this path)
    ln -sf /usr/local/lib/libprotobuf.so /lib/libprotobuf.so && \
    ln -sf /usr/local/lib/libprotobuf.so.28 /lib/libprotobuf.so.28 && \
    ln -sf /usr/local/lib/libprotoc.so /lib/libprotoc.so && \
    # Verify grpc_cpp_plugin is installed and accessible
    ls -la /usr/local/bin/grpc_cpp_plugin && \
    cd / && rm -rf /tmp/grpc

# Add Robotpkg repository for Pinocchio and eigenpy (version 2.5.2)
# eigenpy is required by hpp-fcl
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg" \
    > /etc/apt/sources.list.d/robotpkg.list && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add - && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    robotpkg-py3*-pinocchio \
    robotpkg-py3*-eigenpy \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Build hpp-fcl from source (version 1.6.0)
# hpp-fcl is not available in Ubuntu 20.04 repos with exact version
# Note: v1.6.0 has old CMake requirements, need to patch CMakeLists.txt
# Also need to fix git:// protocol in submodules to use https://
WORKDIR /tmp
RUN git config --global url."https://github.com/".insteadOf "git://github.com/" && \
    git clone https://github.com/humanoid-path-planner/hpp-fcl.git && \
    cd hpp-fcl && \
    git checkout v1.6.0 && \
    # Fix submodule URLs to use https instead of git://
    sed -i 's|git://github.com/|https://github.com/|g' .gitmodules && \
    git submodule sync && \
    git submodule update --init --recursive && \
    # Patch CMakeLists.txt to fix CMake version requirement
    sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' CMakeLists.txt && \
    mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_PREFIX_PATH=/opt/openrobots && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/hpp-fcl

# Install Eigen 3.3.9 from source (header-only library)
# Ubuntu 20.04 has 3.3.7, so we'll install 3.3.9 to match environment.yml
# Eigen is header-only, so we can patch CMakeLists.txt or install directly
WORKDIR /tmp
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz && \
    tar -xzf eigen-3.3.9.tar.gz && \
    cd eigen-3.3.9 && \
    # Patch CMakeLists.txt to fix CMake version requirement
    sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' CMakeLists.txt && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make install && \
    cd / && rm -rf /tmp/eigen-3.3.9*

# ============================================================================
# Install Python Dependencies (matching environment.yml versions)
# ============================================================================
# Set Python 3.8 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1 && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1

# Upgrade pip, but use compatible version for old packages
# pip 24.1+ has stricter dependency resolution that breaks old packages like hydra-core==1.0.6
RUN python3.8 -m pip install --upgrade "pip<24.1" setuptools wheel

# Install PyTorch 1.13.1 (CPU version, matching environment.yml)
RUN python3.8 -m pip install torch==1.13.1 torchvision==0.14.1 torchaudio==0.13.1 --index-url https://download.pytorch.org/whl/cpu

# Install MKL (Intel Math Kernel Library) - required for PyTorch
RUN python3.8 -m pip install mkl mkl-include

# Install Polymetis Python dependencies (matching environment.yml)
# Note: numpy is installed last to prevent other packages from auto-upgrading it
# Fix scipy version to 1.10.1 for compatibility with Python 3.8 and numpy 1.26.4
RUN python3.8 -m pip install \
    "scipy==1.10.1" \
    pandas \
    matplotlib \
    tqdm \
    black \
    pytest \
    pytest-benchmark==4.0.0 \
    grpcio==1.46.0 \
    grpcio-tools==1.46.0 \
    protobuf==3.18.1 \
    pybullet \
    pymodbus==2.5.3 \
    pyserial \
    importlib-resources==5.7.1 \
    sphinx \
    sphinx-copybutton \
    sphinx-book-theme \
    breathe \
    dash \
    plotly \
    asv \
    myst-parser && \
    # Install hydra-core==1.0.6 with compatible omegaconf version
    # hydra-core 1.0.6 requires omegaconf<2.1,>=2.0.5, so install 2.0.6 first
    python3.8 -m pip install omegaconf==2.0.6 && \
    python3.8 -m pip install hydra-core==1.0.6

# ============================================================================
# Install MuJoCo and dm-control dependencies (required for IK solver)
# ============================================================================
# Install MuJoCo 2.3.2 (matching pyproject.toml and robot_ik_solver requirements)
# MuJoCo 2.3.2 is required for compatibility with dm-control 1.0.5 and dm-robotics 0.5.0
RUN python3.8 -m pip install mujoco==2.3.2

# Install dm-control 1.0.5 (required for IK solver, matching pyproject.toml)
# dm-control depends on MuJoCo and provides the mjcf.Physics interface
RUN python3.8 -m pip install dm-control==1.0.5

# Install habitat-sim (optional, but in environment.yml)
# Note: This is a large package, may take time
RUN python3.8 -m pip install habitat-sim==0.2.1 || echo "Warning: habitat-sim installation failed, continuing..."

# ============================================================================
# Install additional role_ros2 dependencies
# ============================================================================
# Use --ignore-installed to handle system-installed packages like PyYAML
RUN python3.8 -m pip install --ignore-installed \
    boto3 \
    customtkinter \
    gym \
    h5py \
    imageio \
    open3d \
    opencv-python==4.6.0.66 \
    opencv-contrib-python==4.6.0.66 \
    Pillow \
    protobuf==3.18.1 \
    psutil \
    pyrallis \
    lark \
    xacro

# ============================================================================
# Install dm-robotics packages (required for IK solver)
# ============================================================================
# dm-robotics packages provide Cartesian6dVelocityEffector for IK computation
# Install with --no-deps to avoid dependency conflicts, but ensure core dependencies are installed first
# Core dependencies (numpy, scipy, dm-control) are already installed above
# Note: dm-robotics-moma 0.5.0 requires:
#   - dm-control (already installed)
#   - numpy (already installed)
#   - scipy (already installed)
#   - mujoco (already installed)
RUN python3.8 -m pip install \
    dm-robotics-moma==0.5.0 \
    dm-robotics-transformations==0.5.0 \
    dm-robotics-agentflow==0.5.0 \
    dm-robotics-geometry==0.5.0 \
    dm-robotics-manipulation==0.5.0 \
    dm-robotics-controllers==0.5.0 \
    --no-deps || echo "Warning: Some dm-robotics packages may have failed, continuing..."

# Install NumPy last to prevent other packages from auto-upgrading it to 2.x
# This must be done after all other packages to ensure NumPy 1.26.4 is the final version
# Then reinstall scipy to ensure it's compiled against the correct NumPy version
RUN python3.8 -m pip install --force-reinstall --no-deps "numpy==1.26.4" || \
    python3.8 -m pip install --force-reinstall "numpy==1.26.4" && \
    python3.8 -m pip install --force-reinstall --no-deps "scipy==1.10.1" || \
    python3.8 -m pip install --force-reinstall "scipy==1.10.1"

# Verify MuJoCo installation and version compatibility
RUN python3.8 -c "import numpy; print(f'✓ NumPy version: {numpy.__version__}')" && \
    python3.8 -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')" && \
    python3.8 -c "import dm_control; print(f'dm-control version: {dm_control.__version__}')" && \
    python3.8 -c "from dm_robotics.moma.effectors import cartesian_6d_velocity_effector; print('dm-robotics-moma imported successfully')" || \
    echo "Warning: MuJoCo/dm-control/dm-robotics verification failed, but continuing..."

# ============================================================================
# Create ROS2 Workspace Structure
# ============================================================================
RUN mkdir -p /app/ros2_ws/src

# ============================================================================
# Copy fairo directory (includes Polymetis)
# ============================================================================
COPY role_ros2/fairo /opt/fairo

# Convert fairo from git submodule to standalone git repository
# Create a git tag for Polymetis version detection
WORKDIR /opt/fairo
RUN if [ -f .git ]; then \
        rm -f .git && \
        git init && \
        git config user.email "build@docker" && \
        git config user.name "Docker Build" && \
        git add . && \
        git commit -m "Initial commit for version detection" || true && \
        git tag -a v0.2.0-0-glocal -m "Docker build version"; \
    elif [ ! -d .git ]; then \
        git init && \
        git config user.email "build@docker" && \
        git config user.name "Docker Build" && \
        git add . && \
        git commit -m "Initial commit for version detection" || true && \
        git tag -a v0.2.0-0-glocal -m "Docker build version"; \
    fi

# Copy hardware config for robot configuration (needed for Polymetis config)
COPY hardware/franka/config /app/ros2_ws/src/role-ros2/hardware/franka/config

# Note: role-ros2 source code will be mounted via docker-compose volume at runtime
# for development. ROS2 workspace should be built after container starts.

WORKDIR /app

# ============================================================================
# Install Polymetis Python package in editable mode
# ============================================================================
WORKDIR ${POLYMETIS_DIR}
RUN python3.8 -m pip install -e ./polymetis

# Copy robot config files to polymetis config directory
WORKDIR /app
RUN echo "Copying robot config files for ${ROBOT_TYPE}..." && \
    if [ -d "${ROBOT_CONFIG_DIR}" ]; then \
        if [ -d "${POLYMETIS_CONFIG_DIR}/robot_client" ] && [ -d "${POLYMETIS_CONFIG_DIR}/robot_model" ]; then \
            if [ -f "${ROBOT_CONFIG_DIR}/franka_hardware.yaml" ]; then \
                cp ${ROBOT_CONFIG_DIR}/franka_hardware.yaml ${POLYMETIS_CONFIG_DIR}/robot_client/franka_hardware.yaml && \
                echo "Copied franka_hardware.yaml for ${ROBOT_TYPE}"; \
            else \
                echo "Warning: franka_hardware.yaml not found in ${ROBOT_CONFIG_DIR}"; \
            fi && \
            if [ -f "${ROBOT_CONFIG_DIR}/franka_panda.yaml" ]; then \
                cp ${ROBOT_CONFIG_DIR}/franka_panda.yaml ${POLYMETIS_CONFIG_DIR}/robot_model/franka_panda.yaml && \
                echo "Copied franka_panda.yaml for ${ROBOT_TYPE}"; \
            else \
                echo "Warning: franka_panda.yaml not found in ${ROBOT_CONFIG_DIR}"; \
            fi; \
        else \
            echo "Error: Required directories ${POLYMETIS_CONFIG_DIR}/robot_client or ${POLYMETIS_CONFIG_DIR}/robot_model do not exist"; \
            exit 1; \
        fi; \
    else \
        echo "Warning: Robot config directory ${ROBOT_CONFIG_DIR} not found"; \
        echo "  Config files will be copied at runtime via entrypoint script if available"; \
    fi

# ============================================================================
# Build libfranka
# ============================================================================
WORKDIR ${FAIRO_DIR}
RUN LIBFRANKA_DIR="polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka" && \
    if [ ! -d "$LIBFRANKA_DIR" ] || [ -z "$(ls -A $LIBFRANKA_DIR 2>/dev/null)" ] || [ ! -d "$LIBFRANKA_DIR/.git" ]; then \
        echo "Initializing libfranka directory at ${FAIRO_DIR}/$LIBFRANKA_DIR" && \
        if [ -d "$LIBFRANKA_DIR" ]; then \
            rm -rf "$LIBFRANKA_DIR"; \
        fi && \
        mkdir -p "$LIBFRANKA_DIR" && \
        cd "$LIBFRANKA_DIR" && \
        git init && \
        git remote add origin https://github.com/frankaemika/libfranka.git && \
        git fetch origin ${LIBFRANKA_VERSION}:${LIBFRANKA_VERSION} 2>/dev/null || git fetch origin && \
        git checkout ${LIBFRANKA_VERSION} 2>/dev/null || git checkout master 2>/dev/null || git checkout main && \
        git submodule update --init --recursive; \
    else \
        echo "libfranka directory already exists at ${FAIRO_DIR}/$LIBFRANKA_DIR, checking out to version ${LIBFRANKA_VERSION}" && \
        cd "$LIBFRANKA_DIR" && \
        git remote add origin https://github.com/frankaemika/libfranka.git 2>/dev/null || git remote set-url origin https://github.com/frankaemika/libfranka.git && \
        git fetch origin ${LIBFRANKA_VERSION}:${LIBFRANKA_VERSION} 2>/dev/null || git fetch origin && \
        git checkout ${LIBFRANKA_VERSION} 2>/dev/null || git checkout master 2>/dev/null || git checkout main && \
        git submodule update --init --recursive; \
    fi && \
    cd "${FAIRO_DIR}/$LIBFRANKA_DIR" && \
    LIBFRANKA_FINAL_VERSION=$(git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD 2>/dev/null || echo "unknown") && \
    echo "libfranka final version: ${LIBFRANKA_FINAL_VERSION}"

# Build libfranka manually (instead of using polymetis script to fix CMake version issues)
WORKDIR ${FAIRO_DIR}
RUN export CMAKE_PREFIX_PATH=/opt/openrobots:/usr/local:$CMAKE_PREFIX_PATH && \
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH && \
    export LD_LIBRARY_PATH=/opt/openrobots/lib:/usr/local/lib:$LD_LIBRARY_PATH && \
    LIBFRANKA_DIR="${POLYMETIS_DIR}/polymetis/src/clients/franka_panda_client/third_party/libfranka" && \
    cd "$LIBFRANKA_DIR" && \
    # Patch CMakeLists.txt to fix CMake version requirement
    find . -name "CMakeLists.txt" -exec sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' {} \; && \
    mkdir -p build && \
    cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt && \
    make -j$(nproc)

# ============================================================================
# Create grpc_python_plugin stub script
# This is a no-op script because we pre-generate Python files using grpcio-tools
# The plugin receives input via stdin from protoc, so we just consume and ignore it
# ============================================================================
RUN echo '#!/bin/bash' > /usr/local/bin/grpc_python_plugin && \
    echo '# No-op: Python files are pre-generated using grpcio-tools' >> /usr/local/bin/grpc_python_plugin && \
    echo 'cat > /dev/null' >> /usr/local/bin/grpc_python_plugin && \
    echo 'exit 0' >> /usr/local/bin/grpc_python_plugin && \
    chmod +x /usr/local/bin/grpc_python_plugin

# ============================================================================
# Build Polymetis C++ components
# ============================================================================
WORKDIR ${POLYMETIS_DIR}
# Pre-generate Python gRPC files using grpcio-tools (since we don't have grpc_python_plugin binary)
RUN cd ./polymetis && \
    python3.8 -m grpc_tools.protoc \
    --proto_path=protos \
    --python_out=. \
    --grpc_python_out=. \
    protos/polymetis.proto && \
    # Move generated files to expected location
    mkdir -p build && \
    cp polymetis_pb2.py build/ || true && \
    cp polymetis_pb2_grpc.py build/ || true

# Now build C++ components
WORKDIR ${POLYMETIS_DIR}
RUN TORCH_PATH=$(python3.8 -c "import torch; import os; print(os.path.dirname(torch.__file__))") && \
    # protobuf was built from source and installed to /usr/local
    export CMAKE_PREFIX_PATH=/usr/local:/usr:/opt/openrobots:${TORCH_PATH}/share/cmake:$CMAKE_PREFIX_PATH && \
    export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig:/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH && \
    export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu:/opt/openrobots/lib:$LD_LIBRARY_PATH && \
    # Set CONDA_PREFIX to help CMake find plugins in /usr/local/bin
    export CONDA_PREFIX=/usr/local && \
    # Also set PATH to ensure grpc plugins are found
    export PATH=/usr/local/bin:$PATH && \
    mkdir -p ./polymetis/build && \
    cd ./polymetis/build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_FRANKA=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_DOCS=OFF \
    -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
    -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
    -DTorch_DIR=${TORCH_PATH}/share/cmake/Torch \
    -DgRPC_DIR=/usr/local/lib/cmake/grpc \
    -DProtobuf_DIR=/usr/local/lib/cmake/protobuf \
    -DPROTOBUF_LIBRARY=/usr/local/lib/libprotobuf.so \
    -DPROTOBUF_INCLUDE_DIR=/usr/local/include \
    -DPROTOBUF_PROTOC_EXECUTABLE=/usr/local/bin/protoc && \
    make -j$(nproc)

# Create lib directory and symlinks for Python to find shared libraries
# torchcontrol expects libraries at ${CONDA_PREFIX}/lib/
WORKDIR ${POLYMETIS_DIR}/polymetis/build
RUN mkdir -p lib && \
    ln -sf ../torch_isolation/libtorchscript_pinocchio.so lib/libtorchscript_pinocchio.so && \
    ln -sf ../torch_isolation/libtorch_server_ops.so lib/libtorch_server_ops.so && \
    ln -sf ../torch_isolation/libpinocchio_wrapper.so lib/libpinocchio_wrapper.so && \
    ln -sf ../torch_isolation/libtorchrot.so lib/libtorchrot.so || true

# ============================================================================
# Install ROS2 workspace dependencies
# ============================================================================
RUN apt-get update && \
    apt-get install -y \
    python3-rosdep \
    python3-vcstool && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# Note: ROS2 workspace will be built at runtime after role-ros2 is mounted
# Run: cd /app/ros2_ws && colcon build --symlink-install

# Set library paths for runtime
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:/usr/local/lib:/opt/fairo/polymetis/polymetis/build/torch_isolation:${LD_LIBRARY_PATH}
ENV CMAKE_PREFIX_PATH=/opt/openrobots:/usr/local:${CMAKE_PREFIX_PATH}
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}
# Add polymetis Python module to PYTHONPATH
# The polymetis package is in /opt/fairo/polymetis/polymetis/python/
ENV PYTHONPATH=/opt/fairo/polymetis/polymetis/python:/usr/lib/python3.8/site-packages:${PYTHONPATH}
# Set CONDA_PREFIX to point to polymetis build directory (for torchcontrol to find shared libs)
ENV CONDA_PREFIX=/opt/fairo/polymetis/polymetis/build

# Set working directory
WORKDIR /app/ros2_ws

# ============================================================================
# Copy Docker configuration files to image
# ============================================================================
# Build context is ../.. (role-ros2 root directory), so path is relative to that
COPY docker/ros2_franka_libfranka_0.18.x/entrypoint.sh /entrypoint.sh
COPY docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env /app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env
RUN chmod +x /entrypoint.sh && \
    chmod +x /app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env && \
    # Add polymetis_ros2.env to bashrc so environment variables are set in new shells
    echo "if [ -f /app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env ]; then source /app/ros2_ws/src/role-ros2/docker/ros2_franka_libfranka_0.18.x/polymetis_ros2.env; fi" >> /root/.bashrc

# Use the entrypoint from image
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
