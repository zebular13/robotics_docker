#!/bin/bash

# Set the base directory where we'll clone/check for the repo
BASE_DIR="/home/$USER"
REPO_DIR="$BASE_DIR/robotics_docker"
WORKSPACE_DIR="$BASE_DIR/robotics_workspace"

# Install sshpass if not already installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass not found. Installing..."
    sudo apt-get update
    sudo apt-get install -y sshpass
    echo "✓ sshpass installed."
else
    echo "sshpass already installed."
fi

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
if [ ! -f /root/hand_controller/hands-v2-yolov5-linux-x86.eim ]; then
    echo "Downloading Edge Impulse model file..."
    cd /root/hand_controller
    wget -O hands-v2-yolov5-linux-x86.eim "https://github.com/zebular13/hand_controller/releases/download/Flask_QIRP1.4/hands-v2-linux-x86.eim"
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
'

echo "Launching the demo in new terminal..."

# Launch Part 2 demo in a new terminal window
gnome-terminal --title="ROS2 Demo Part 2" -- bash -c "
docker compose -f $REPO_DIR/compose/docker-compose.yml exec robotics_demo bash -ic '
cd /root/hand_controller/ros2_ws && 
source install/setup.bash && 
ros2 launch hand_controller demo11_mogiros_car_part2.launch.py use_imshow:=False
'; exec bash"

echo "Docker demo setup complete!"