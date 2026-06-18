FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

WORKDIR /manifold_ws

COPY src/ ./src/
COPY models/ ./models/
COPY walk.dae ./walk.dae

# Install dependencies
RUN apt-get update && apt-get install -y git wget \
    ros-noetic-costmap-2d ros-noetic-octomap-msgs ros-noetic-rviz-visual-tools libcgal-dev ros-noetic-base-local-planner ros-noetic-move-base libignition-math4-dev \
    ros-noetic-sick-tim ros-noetic-lms1xx ros-noetic-interactive-marker-twist-server ros-noetic-robot-localization ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-velodyne-description ros-noetic-pointgrey-camera-description ros-noetic-twist-mux \
    ros-noetic-realsense2-description ros-noetic-pointcloud-to-laserscan ros-noetic-jsk-recognition ros-noetic-jsk-visualization \
    && rm -rf /var/lib/apt/lists/*

# Upgrade CMake
RUN cd /tmp \
    && wget https://github.com/Kitware/CMake/releases/download/v3.22.6/cmake-3.22.6-linux-x86_64.sh \
    && chmod +x cmake-3.22.6-linux-x86_64.sh \
    && sudo ./cmake-3.22.6-linux-x86_64.sh --prefix=/usr/local --skip-license \
    && hash -r

# Install OSQP
RUN cd /tmp \
    && git clone --recursive https://github.com/osqp/osqp \
    && cd osqp \
    && mkdir build && cd build \
    && cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON .. \
    && cmake --build . \
    && cmake --build . --target install

# Install OSQP-Eigen
RUN cd /tmp \ 
    && git clone https://github.com/robotology/osqp-eigen.git \ 
    && cd osqp-eigen \ 
    && mkdir build && cd build \ 
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \ 
    && make -j$(nproc) \ 
    && make install

# Install LPFGSpp
RUN cd /tmp \
    && git clone https://github.com/yixuan/LBFGSpp.git \
    && cp -r LBFGSpp/include/* /usr/local/include/

# Build workspace
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17

# Automatically source for each opened terminal
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /manifold_ws/devel/setup.bash" >> ~/.bashrc

CMD ["bash"]
