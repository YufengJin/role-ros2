#!/bin/bash
# Quick rebuild script for ROS2 workspace
# Usage: ./quick_rebuild.sh [package_name]
#   - If package_name is provided, build only that package
#   - If no package_name, build entire workspace

set -e  # Exit on error

# Store current directory
ORIGINAL_DIR=$(pwd)

# Workspace directory
WORKSPACE_DIR="/app/ros2_ws"

# Check if workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ Error: Workspace directory not found: $WORKSPACE_DIR"
    exit 1
fi

# Change to workspace directory
echo "📂 Changing to workspace: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

# Check if package name is provided
if [ -n "$1" ]; then
    PACKAGE_NAME="$1"
    echo "🔨 Building package: $PACKAGE_NAME"
    colcon build --symlink-install --packages-select "$PACKAGE_NAME"
else
    echo "🔨 Building entire workspace..."
    colcon build --symlink-install
fi

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully"
    
    # Source the setup file
    if [ -f "install/setup.bash" ]; then
        echo "📦 Sourcing install/setup.bash..."
        source install/setup.bash
        echo "✅ Setup file sourced"
    else
        echo "⚠️  Warning: install/setup.bash not found"
    fi
else
    echo "❌ Build failed"
    cd "$ORIGINAL_DIR"
    exit 1
fi

# Return to original directory
echo "📂 Returning to original directory: $ORIGINAL_DIR"
cd "$ORIGINAL_DIR"

echo "✅ Done!"
