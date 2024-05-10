FROM ros:noetic-ros-core-focal

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Update the GPG key for the ROS repository and install necessary packages
RUN apt-get update && apt-get install -y gnupg2 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y \
    ros-noetic-image-view \
    ros-noetic-cv-bridge \
    python3-pip \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-msgs && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    rosdep \
    rosinstall_generator \
    wstool \
    rosinstall \
    opencv-python-headless \
    PyYAML

# Setup rosdep
RUN rosdep init && rosdep update

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh

# Make entrypoint script executable
RUN chmod +x /entrypoint.sh

# Use entrypoint script to setup ROS environment
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Revert to user interaction for apt-get and other commands
ENV DEBIAN_FRONTEND=dialog
