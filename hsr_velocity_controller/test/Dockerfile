FROM osrf/ros:humble-desktop-full

# Install build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
RUN rosdep update

WORKDIR /root/ws

# Copy package.xml to install dependencies
COPY src/iai_hsr/hsr_velocity_controller/package.xml src/iai_hsr/hsr_velocity_controller/package.xml

# Install dependencies
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

# Add source command to bashrc for interactive sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
