#!/bin/bash
# Quick build script for role_ros2 package
# 
# This script works in both Docker container and host PC:
#   - Container: /app/ros2_ws/src/role-ros2/quick_build.sh
#   - Host PC:    /home/yjin/repos/ros2_ws/src/role-ros2/quick_build.sh
# 
# Usage: ./quick_build.sh [options]
# 
# Options:
#   --clean      Clean build directory before building
#   --symlink    Use symlink install mode (for development)
#   --help       Show this help message
#
# Examples:
#   # Basic build
#   ./quick_build.sh
#
#   # Clean build
#   ./quick_build.sh --clean
#
#   # Development mode (symlink install)
#   ./quick_build.sh --symlink

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGE_NAME="role_ros2"

# Parse arguments
CLEAN_BUILD=false
SYMLINK_INSTALL=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --symlink)
            SYMLINK_INSTALL=true
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --clean      Clean build directory before building"
            echo "  --symlink    Use symlink install mode (for development)"
            echo "  --help       Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Quick Build Script for ${PACKAGE_NAME}${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to workspace root
echo -e "${YELLOW}[1/4]${NC} Changing to workspace root: ${WS_ROOT}"

# Detect environment (container vs host)
if [[ "$WS_ROOT" == "/app/ros2_ws" ]]; then
    echo -e "${BLUE}   Environment: Docker container${NC}"
elif [[ "$WS_ROOT" == *"ros2_ws"* ]]; then
    echo -e "${BLUE}   Environment: Host PC${NC}"
fi

cd "$WS_ROOT"

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}[2/4]${NC} Cleaning build directory..."
    rm -rf build/$PACKAGE_NAME install/$PACKAGE_NAME log/latest_$PACKAGE_NAME
    echo -e "${GREEN}✓${NC} Clean complete"
else
    echo -e "${YELLOW}[2/4]${NC} Skipping clean (use --clean to clean before build)"
fi

# Build package
echo -e "${YELLOW}[3/4]${NC} Building package: ${PACKAGE_NAME}"

BUILD_CMD="colcon build --packages-select $PACKAGE_NAME"
if [ "$SYMLINK_INSTALL" = true ]; then
    BUILD_CMD="$BUILD_CMD --symlink-install"
    echo -e "${BLUE}   Using symlink install mode${NC}"
fi

echo "   Command: $BUILD_CMD"
echo ""

if $BUILD_CMD; then
    echo -e "${GREEN}✓${NC} Build successful"
else
    echo -e "${RED}✗${NC} Build failed!"
    exit 1
fi

# Source setup.bash
echo ""
echo -e "${YELLOW}[4/4]${NC} Sourcing install/setup.bash..."

if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
    echo -e "${GREEN}✓${NC} Environment sourced successfully"
else
    echo -e "${RED}✗${NC} setup.bash not found! Build may have failed."
    exit 1
fi

# Summary
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Package: ${PACKAGE_NAME}"
echo "Workspace: ${WS_ROOT}"
echo "Install location: ${WS_ROOT}/install/${PACKAGE_NAME}"
echo ""

