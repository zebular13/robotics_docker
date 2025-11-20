#!/bin/bash

# Vision AI Kit configuration
VISION_KIT_IP="192.168.1.2" # Change this to your actual IP 
VISION_KIT_PASSWORD="oelinux123"

# Install sshpass if not already installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass not found. Installing..."
    sudo apt-get update
    sudo apt-get install -y sshpass
    echo "✓ sshpass installed."
else
    echo "sshpass already installed."
fi

# Setup Vision AI Kit (QC6490 board)
echo "Setting up Vision AI Kit (QC6490 board)..."

# Check if we can reach the board
if ping -c 1 -W 2 $VISION_KIT_IP &> /dev/null; then
    echo "Vision AI Kit detected at $VISION_KIT_IP"

    # Check if model file exists on Vision AI Kit
    MODEL_EXISTS=$(sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP "[ -f /root/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim ] && echo 'yes' || echo 'no'")

    if [ "$MODEL_EXISTS" = "no" ]; then
        echo "Model file not found on Vision AI Kit."
        
        # Download on host machine if needed
        if [ ! -f /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim ]; then
            echo "Downloading Edge Impulse model file on host machine..."
            wget -O /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim \
                "https://github.com/zebular13/hand_controller/releases/download/Flask_QIRP1.4/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim"
            
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
        
        # Copy to Vision AI Kit
        echo "Copying model file to Vision AI Kit..."
        sshpass -p "$VISION_KIT_PASSWORD" scp /tmp/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim \
            root@$VISION_KIT_IP:/root/
        echo "Model file transferred successfully."
    else
        echo "Model file already exists on Vision AI Kit."
    fi
    
    # Now run the main setup in a single SSH session
    sshpass -p "$VISION_KIT_PASSWORD" ssh -o StrictHostKeyChecking=no root@$VISION_KIT_IP 'bash -s' << 'ENDSSH'
    # Re-mount /usr as writable
    mount -o remount,rw /usr

    # Check and install colcon if not present
    echo "Checking for colcon..."
    if ! command -v colcon &> /dev/null; then
        echo "colcon not found. Installing colcon-common-extensions via pip3..."
        pip3 install colcon-common-extensions
        
        # Verify installation
        if command -v colcon &> /dev/null; then
            echo "✓ colcon installed successfully"
            echo "colcon version: $(colcon --version 2>/dev/null || echo 'version check failed')"
        else
            echo "⚠ Warning: colcon installation may have failed"
            echo "Trying alternative installation method..."
            pip3 install --user colcon-common-extensions
            export PATH="$PATH:/root/.local/bin"
        fi
    else
        echo "colcon already installed."
        echo "colcon version: $(colcon --version 2>/dev/null || echo 'version check failed')"
    fi

    # Install edge_impulse_linux to system site-packages
    echo "Checking for edge_impulse_linux package..."
    if python3 -c "import sys; import edge_impulse_linux; print(edge_impulse_linux.__file__)" 2>/dev/null | grep -q "/usr/lib/python3.12/site-packages"; then
        echo "edge_impulse_linux already installed in system site-packages."
    else
        echo "Installing edge_impulse_linux to system site-packages..."
        pip3 install --target=/usr/lib/python3.12/site-packages edge_impulse_linux
        
        # Verify installation in correct location
        if python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3.12/site-packages'); import edge_impulse_linux; print('Success')" 2>/dev/null | grep -q "Success"; then
            echo "✓ edge_impulse_linux installed successfully in /usr/lib/python3.12/site-packages"
        else
            echo "⚠ Warning: edge_impulse_linux installation verification failed."
        fi

        # # Fix edge_impulse_linux by commenting out pyaudio - NO LONGER NECESSARY WITH UPDATED IMAGE THAT CONTAINS PYAUDIO
        # echo "Fixing edge_impulse_linux package..."
        # if [ -f /usr/lib/python3.12/site-packages/edge_impulse_linux/__init__.py ]; then
        #     sed -i 's/^from edge_impulse_linux import audio/#from edge_impulse_linux import audio/' \
        #         /usr/lib/python3.12/site-packages/edge_impulse_linux/__init__.py
        #     echo "✓ edge_impulse_linux package fixed."
        # else
        #     echo "⚠ Warning: Could not find __init__.py to fix."
        # fi
    fi
    
    # Install flask if not already installed in system site-packages
    echo "Checking for flask package..."
    FLASK_CHECK=$(python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3.12/site-packages'); import flask; print(flask.__file__)" 2>/dev/null)

    if echo "$FLASK_CHECK" | grep -q "/usr/lib/python3.12/site-packages"; then
        echo "flask already installed in system site-packages."
        FLASK_VERSION=$(python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3.12/site-packages'); import flask; print(flask.__version__)" 2>/dev/null)
        echo "flask version: $FLASK_VERSION"
    else
        echo "flask not found in system site-packages. Installing..."
        pip3 install --target=/usr/lib/python3.12/site-packages flask==3.0.0 
        
        # Verify installation
        FLASK_VERIFY=$(python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3.12/site-packages'); import flask; print(flask.__file__)" 2>/dev/null)
        if echo "$FLASK_VERIFY" | grep -q "/usr/lib/python3.12/site-packages"; then
            FLASK_VERSION=$(python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3.12/site-packages'); import flask; print(flask.__version__)" 2>/dev/null)
            echo "✓ flask $FLASK_VERSION installed successfully in system site-packages."
        else
            echo "⚠ Warning: flask installation verification failed."
            echo "Attempted import returned: $FLASK_VERIFY"
        fi
    fi

    # Clone hand_controller if it doesn't exist or is incomplete
    echo "Checking hand_controller repository..."
    if [ ! -d /root/hand_controller ]; then
        echo "hand_controller repository not found. Cloning..."
        cd /root
        git clone --recursive https://github.com/AlbertaBeef/hand_controller
        echo "Repository cloned."
    elif [ ! -d /root/hand_controller/.git ]; then
        echo "hand_controller directory exists but is not a git repository. Re-cloning..."
        rm -rf /root/hand_controller
        cd /root
        git clone --recursive https://github.com/AlbertaBeef/hand_controller
        echo "Repository cloned."
    else
        echo "hand_controller repository exists. Checking integrity..."
        cd /root/hand_controller
        
        # Check if it's a valid git repo and fetch latest
        if git rev-parse --git-dir > /dev/null 2>&1; then
            # Check for essential directories/files
            if [ ! -d "ros2_ws" ]; then
                echo "Repository appears incomplete. Re-cloning..."
                cd /root
                rm -rf hand_controller
                git clone --recursive https://github.com/AlbertaBeef/hand_controller
                echo "Repository cloned."
            else
                # Update submodules if needed
                echo "Updating repository and submodules..."
                git pull
                git submodule update --init --recursive
                echo "✓ Repository up to date."
            fi
        else
            echo "Invalid git repository. Re-cloning..."
            cd /root
            rm -rf hand_controller
            git clone --recursive https://github.com/AlbertaBeef/hand_controller
            echo "Repository cloned."
        fi
    fi

    # Move model file to correct location and set permissions
    mv /root/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim /root/hand_controller/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim
    chmod +x /root/hand_controller/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim

    # Final verification
    if [ -d /root/hand_controller/ros2_ws ] && [ -d /root/hand_controller/launch ]; then
        echo "✓ hand_controller repository verified."
    else
        echo "⚠ Warning: hand_controller repository may be incomplete."
        echo "Contents of /root/hand_controller:"
        ls -la /root/hand_controller/
    fi
    
    # Initialize ROS environment (ALWAYS)
    echo "Initializing ROS environment..."
    export QMONITOR_BACKEND_LIB_PATH=/var/QualcommProfiler/libs/backends/ 
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/var/QualcommProfiler/libs/ 
    export PATH=$PATH:/data/shared/QualcommProfiler/bins 
    export XDG_RUNTIME_DIR=/dev/socket/weston 
    export WAYLAND_DISPLAY=wayland-1 
    export HOME=/home
    export ROS_DOMAIN_ID=0
    cd /root/hand_controller/ros2_ws 
    colcon build --packages-select hand_controller
    source install/setup.bash 
    
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
        model:=/root/hand_controller/hands-v2-yolov5-conferencedata-aarch64-qnn-v42.eim \
        use_flask:=True
    
ENDSSH

else
    echo "Warning: Vision AI Kit not detected at $VISION_KIT_IP"
    echo "Skipping Vision AI Kit setup. Make sure the board is connected."
fi