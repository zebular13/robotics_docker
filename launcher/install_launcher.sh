#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing Robotics Demo Launcher..."
echo "Script directory: $SCRIPT_DIR"

# 1. Copy and make launch script executable
echo "Installing launch script..."
cp "$SCRIPT_DIR/launch_docker.sh" ~/launch_docker.sh
chmod +x ~/launch_docker.sh
echo "✓ Launch script installed to ~/launch_docker.sh"

# 2. Update the .desktop file with correct path and install it
echo "Installing desktop launcher..."
# Create a temporary .desktop file with updated path
sed "s|Exec=gnome-terminal -- bash -c \"/home/monica-ssd/launch_docker.sh|Exec=gnome-terminal -- bash -c \"$HOME/launch_docker.sh|g" \
    "$SCRIPT_DIR/RoboticsDemo.desktop" > /tmp/RoboticsDemo.desktop

# Copy to applications directory and make executable
mkdir -p ~/.local/share/applications
cp /tmp/RoboticsDemo.desktop ~/.local/share/applications/RoboticsDemo.desktop
chmod +x ~/.local/share/applications/RoboticsDemo.desktop
rm /tmp/RoboticsDemo.desktop
echo "✓ Desktop launcher installed to ~/.local/share/applications/RoboticsDemo.desktop"

# 3. Install custom icon if it exists
if [ -f "$SCRIPT_DIR/robotics-demo.png" ]; then
    echo "Installing custom icon..."
    mkdir -p ~/.local/share/icons/hicolor/256x256/apps
    cp "$SCRIPT_DIR/robotics-demo.png" ~/.local/share/icons/hicolor/256x256/apps/robotics-demo.png
    echo "✓ Custom icon installed"
    
    # Update icon cache
    echo "Updating icon cache..."
    gtk-update-icon-cache -f -t ~/.local/share/icons/hicolor/ 2>/dev/null
else
    echo "⚠ Custom icon (robotics-demo.png) not found in $SCRIPT_DIR"
    echo "  The launcher will use a default system icon."
fi

# 4. Update desktop database
echo "Updating desktop database..."
update-desktop-database ~/.local/share/applications/

echo ""
echo "✓ Installation complete!"
echo ""
echo "You can now:"
echo "  1. Find 'Robotics Demo' in your applications menu"
echo "  2. Run it directly: ~/launch_docker.sh"
echo ""
echo "Note: If the icon doesn't appear immediately, try:"
echo "  - Logging out and back in"
echo "  - Or press Alt+F2, type 'r', and press Enter (GNOME only)"