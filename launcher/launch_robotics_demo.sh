#!/bin/bash

# Set the base directory where we'll clone/check for the repo
BASE_DIR="/home/$USER"
REPO_DIR="$BASE_DIR/robotics_docker"
WORKSPACE_DIR="$BASE_DIR/robotics_workspace"

# Vision AI Kit configuration
VISION_KIT_IP="192.168.1.2" # Change this to your actual IP 
VISION_KIT_PASSWORD="oelinux123"  

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
'

echo "Launching the demo in new terminal..."

# Launch Part 2 demo in a new terminal window
gnome-terminal --title="ROS2 Demo Part 2" -- bash -c "
docker compose -f $REPO_DIR/compose/docker-compose.yml exec robotics_demo bash -ic '
cd /root/hand_controller/ros2_ws && 
source install/setup.bash && 
ros2 launch hand_controller demo11_mogiros_car_part2.launch.py use_imshow:=False
'; exec bash"

# Wait a moment before starting Vision AI Kit setup
sleep 3

# Setup Vision AI Kit (QC6490 board)
echo "Setting up Vision AI Kit (QC6490 board)..."

# Check if we can reach the board
if ping -c 1 -W 2 $VISION_KIT_IP &> /dev/null; then
    echo "Vision AI Kit detected at $VISION_KIT_IP"
    
    # Check if ROS package exists on Vision AI Kit
    ROS_PACKAGE_EXISTS=$(sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP "[ -f /opt/hand_controller_ros2_qsc6490.tar.gz ] && echo 'yes' || echo 'no'")
    
    if [ "$ROS_PACKAGE_EXISTS" = "no" ]; then
        echo "ROS2 package not found on Vision AI Kit."
        
        # Download on host machine if needed
        if [ ! -f /tmp/hand_controller_ros2_qsc6490.tar.gz ]; then
            echo "Downloading ROS2 package on host machine..."
            wget -O /tmp/hand_controller_ros2_qsc6490.tar.gz \
                "https://avtinc.sharepoint.com/:u:/t/ET-Downloads/EbMBA64GB35Bjwq5wMnYi0UBP_RiqKUYtWK3QhVd7Wzgtw?e=8jCubK&download=1"
            
            # Verify file size (should be around 78.7 MB = ~82,000,000 bytes)
            FILE_SIZE=$(stat -c%s /tmp/hand_controller_ros2_qsc6490.tar.gz 2>/dev/null || stat -f%z /tmp/hand_controller_ros2_qsc6490.tar.gz 2>/dev/null)
            if [ "$FILE_SIZE" -lt 10000000 ]; then
                echo "Error: Downloaded file is too small ($FILE_SIZE bytes). Download may have failed."
                rm /tmp/hand_controller_ros2_qsc6490.tar.gz
                exit 1
            fi
            echo "Download complete. File size: $FILE_SIZE bytes"
        else
            echo "ROS2 package already downloaded on host."
        fi
        
        # Copy to Vision AI Kit
        echo "Copying ROS2 package to Vision AI Kit..."
        sshpass -p "$VISION_KIT_PASSWORD" scp /tmp/hand_controller_ros2_qsc6490.tar.gz \
            root@$VISION_KIT_IP:/opt/
        echo "Copy complete."
    else
        echo "ROS2 package already exists on Vision AI Kit."
    fi

    # Check if model file exists on Vision AI Kit
    MODEL_EXISTS=$(sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP "[ -f /root/hand_controller/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim ] && echo 'yes' || echo 'no'")

    if [ "$MODEL_EXISTS" = "no" ]; then
        echo "Model file not found on Vision AI Kit."
        
        # Download on host machine if needed
        if [ ! -f /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim ]; then
            echo "Downloading Edge Impulse model file on host machine..."
            wget -O /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim \
                "https://avtinc.sharepoint.com/:u:/t/ET-Downloads/ER93CZ3Nc8xMomCf9uGyBe8BruM9h48BBru-8s3X-38djQ?e=ZCOr7e&download=1"
            
            # Verify file size (should be reasonably large, not an error page)
            FILE_SIZE=$(stat -c%s /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim 2>/dev/null || stat -f%z /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim 2>/dev/null)
            if [ "$FILE_SIZE" -lt 100000 ]; then
                echo "Error: Downloaded model file is too small ($FILE_SIZE bytes). Download may have failed."
                rm /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim
                exit 1
            fi
            echo "Model download complete. File size: $FILE_SIZE bytes"
        else
            echo "Model file already downloaded on host."
        fi
        
        # Ensure hand_controller directory exists on Vision AI Kit
        echo "Ensuring hand_controller directory exists..."
        sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP "mkdir -p /root/hand_controller"
        
        # Copy to Vision AI Kit
        echo "Copying model file to Vision AI Kit..."
        sshpass -p "$VISION_KIT_PASSWORD" scp /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim \
            root@$VISION_KIT_IP:/root/hand_controller/
        
        # Make executable
        sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP \
            "chmod +x /root/hand_controller/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim"
        
        echo "Model file transferred successfully."
    else
        echo "Model file already exists on Vision AI Kit."
        # Make sure it's executable
        sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP \
            "chmod +x /root/hand_controller/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim"
    fi
    
    # Now run the main setup in a single SSH session
    sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP 'bash -s' << 'ENDSSH'
    
    # Check and install Edge Impulse setup script
    if [ ! -d edge-impulse-tools ]; then
        echo "Edge Impulse tools not found. Installing..."
        
        # Download setup script if it doesn't exist
        if [ ! -f /root/setup-edge-impulse-qc-linux.sh ]; then
            echo "Downloading Edge Impulse setup script..."
            wget https://cdn.edgeimpulse.com/firmware/linux/setup-edge-impulse-qc-linux.sh
        fi
        
        # Run the setup script
        echo "Running Edge Impulse setup script..."
        sh setup-edge-impulse-qc-linux.sh
    else
        echo "Edge Impulse tools already installed."
    fi
    
    echo "Installing edge_impulse_linux pip package..."
    pip3 install edge_impulse_linux

    # Fix edge_impulse_linux by commenting out pyaudio
    echo "Fixing edge_impulse_linux package..."
    sed -i 's/^from edge_impulse_linux import audio/#from edge_impulse_linux import audio/' \
        /root/.local/lib/python3.12/site-packages/edge_impulse_linux/__init__.py
    echo "edge_impulse_linux installed."
    
    echo "Installing flask pip package..."
    pip3 install flask==3.0.0
    echo "flask installed."

    # Clone hand_controller if it doesn't exist
    if [ ! -d /root/hand_controller ]; then
        echo "Cloning hand_controller repository..."
        cd /root
        git clone --recursive https://github.com/AlbertaBeef/hand_controller
    else
        echo "hand_controller repository already exists."
    fi
    
    # Check if ROS2 package needs to be extracted
    ROS_PACKAGE_DOWNLOADED=false
    if [ -f /opt/hand_controller_ros2_qsc6490.tar.gz ]; then
        # Check if already extracted by looking for the package
        if [ ! -d /usr/share/hand_controller ]; then
            ROS_PACKAGE_DOWNLOADED=true
        fi
    fi
    
    # Extract ROS2 package if needed
    if [ "$ROS_PACKAGE_DOWNLOADED" = true ]; then
        echo "Extracting ROS2 package..."
        # Re-mount /usr as writable
        mount -o remount,rw /usr
        
        # Extract tarball to /usr with error checking
        if tar -zxf /opt/hand_controller_ros2_qsc6490.tar.gz -C /usr/; then
            echo "ROS2 package extracted successfully."
            # Wait for filesystem to sync
            sync
            sleep 2
        else
            echo "Error: Failed to extract ROS2 package!"
            exit 1
        fi
    else
        echo "ROS2 package already extracted."
    fi
    
    # Initialize ROS environment (ALWAYS)
    echo "Initializing ROS environment..."
    export HOME=/home
    export ROS_DOMAIN_ID=0
    source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
    
    # Verify hand_controller ROS2 package
    echo "Verifying hand_controller ROS2 package..."
    PACKAGE_CHECK=$(ros2 pkg executables hand_controller 2>&1)
    echo "$PACKAGE_CHECK"
    
    if echo "$PACKAGE_CHECK" | grep -q "hand_controller_ei1dials_twist_node"; then
        echo "✓ hand_controller package verified successfully!"
    else
        echo "⚠ Warning: hand_controller package may not be fully installed."
        echo "Expected executables not found."
    fi
    
    # Launch the demo
    echo "Launching Vision AI Kit demo..."
    ros2 launch hand_controller demo11_mogiros_car_part1_ei1dials.launch.py \
        verbose:=False \
        model:=/root/hand_controller/hands-v2-yolov5-linux-aarch64-qnn-v36.eim \
        use_flask:=True
    
ENDSSH

else
    echo "Warning: Vision AI Kit not detected at $VISION_KIT_IP"
    echo "Skipping Vision AI Kit setup. Make sure the board is connected."
fi