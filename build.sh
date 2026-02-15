#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing package dependencies..."
rosdep install -i --from-path src --rosdistro humble -y

# Build the workspace
echo "Building delivery robot workspace..."
colcon build --symlink-install --parallel-workers 4

# Check build status
if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Run 'source install/setup.bash' to use the packages"
else
    echo "Build failed! Check the errors above."
    exit 1
fi
