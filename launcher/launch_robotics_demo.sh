#!/bin/bash

# Set the base directory where we'll clone/check for the repo
BASE_DIR="/home/$USER"
REPO_DIR="$BASE_DIR/robotics_docker"
WORKSPACE_DIR="$BASE_DIR/robotics_workspace"

BOARD_IP="192.168.1.2" # Update with your board's IP if different
PASSWORD="oelinux123"

# Check if robotics_docker repo exists, if not clone it
echo "Checking for robotics_docker repository..."
if [ ! -d "$REPO_DIR" ]; then
    echo "Repository not found. Cloning robotics_docker..."
    cd "$BASE_DIR"
    git clone https://github.com/AlbertaBeef/robotics_docker.git
    echo "Repository cloned successfully!"
    
    # Create persistent storage directory
    echo "Creating persistent storage directory..."
    mkdir -p "$WORKSPACE_DIR"
    
    # Update docker-compose.yml with persistent storage path
    echo "Configuring docker-compose.yml for persistent storage..."
    sed -i "s|/media/albertabeef/Tycho/ROS2/shared/ros2|$WORKSPACE_DIR|g" "$REPO_DIR/compose/docker-compose.yml"
    echo "docker-compose.yml updated!"
else
    echo "Repository already exists at $REPO_DIR"
fi

# Pull the latest Docker image
echo "Pulling latest Docker image..."
docker pull albertabeef/robotics_docker:latest

# Navigate to the compose directory
cd "$REPO_DIR/compose"

# Launch the docker container
echo "Starting Docker container..."
docker compose up -d robotics_demo

# Wait a moment for container to be ready
sleep 2

# Allow X-Windows on host
echo "Configuring X-Windows access..."
xhost +

# Install Python dependencies if not already installed
echo "Checking Python dependencies..."
docker compose exec robotics_demo bash -ic '
if ! python3 -c "import flask" 2>/dev/null || ! python3 -c "import edge_impulse_linux" 2>/dev/null; then
    echo "Installing missing Python packages..."
    pip3 install flask==3.0.0 edge_impulse_linux
else
    echo "Python dependencies already installed."
fi
'

# Download Edge Impulse model file if it doesn't exist
echo "Checking for Edge Impulse model file..."
docker compose exec robotics_demo bash -ic '
if [ ! -f /root/hand_controller/hands-v2-yolov5-linux-amd86-v39new.eim ]; then
    echo "Downloading Edge Impulse model file..."
    cd /root/hand_controller
    wget -O hands-v2-yolov5-linux-x86.eim "https://avtinc.sharepoint.com/:u:/t/ET-Downloads/Ec5lhZN6XpZCug51d3brcWMBgfKSJgnYKpF1zopW1LUNqg?e=sgoHmO&download=1"
    echo "Model file downloaded successfully!"
else
    echo "Edge Impulse model file already exists."
fi
'

# Check if workspace is already built, if not build it
echo "Checking workspace setup..."
docker compose exec robotics_demo bash -ic '
if [ ! -f /root/hand_controller/ros2_ws/install/setup.bash ]; then
    echo "First time setup - building workspace..."
    cd /root/hand_controller
    git pull
    cd ros2_ws
    colcon build
    echo "source /root/hand_controller/ros2_ws/install/setup.bash >> /root/.bashrc" 
    echo "Workspace built successfully!"
else
    echo "Workspace already built, skipping build step."
fi
echo "Launching the demo..."
ros2 launch hand_controller demo11_mogiros_car_part2.launch.py use_imshow:=False 
'


# Commands to execute on the remote board
echo "Launching demo on Vision AI Kit 6490.."
REMOTE_COMMANDS=$(cat << 'EOF'
export HOME=/home
export ROS_DOMAIN_ID=0
source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
ros2 pkg executables | grep hand_controller
ros2 launch hand_controller demo11_mogiros_car_part1_ei1dials.launch.py verbose:=False model:=/root/hand_controller/hands-v2-yolov5-linux-aarch64-qnn-v36.eim use_flask:=True
EOF
)

# Execute commands remotely using sshpass
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no root@$BOARD_IP "$REMOTE_COMMANDS"