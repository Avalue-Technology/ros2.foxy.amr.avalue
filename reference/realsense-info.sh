#!/bin/bash

# sudo apt update
# sudo apt install v4l-utils

echo "Intel® RealSense™ - Video Mapping"
echo

for dev in /dev/video*; do
    # Get sysfs names (less prone to truncation than v4l2-ctl)
    name=$(cat /sys/class/video4linux/$(basename $dev)/name 2>/dev/null)
    [ -z "$name" ] && continue

    # Get Supported Image Formats
    formats=$(v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | grep -E "'(Z16|Y8 |Y16|YUYV|MJPG)'")

    # Evaluate Camera Functionality
    if echo "$formats" | grep -q "MJPG\|YUYV"; then
        role="RGB Color Camera"
    elif echo "$formats" | grep -q "Z16"; then
        role="Depth Camera"
    elif echo "$formats" | grep -q "Y8"; then
        role="Infrared Camera"
    else
        role="Unknown/Metadata Interface"
    fi

    echo "$dev"
    echo "  Name   : $name"
    echo "  Formats: $(echo "$formats" | tr '\n' ' ' | sed 's/[[:space:]]\+/ /g')"
    echo "  Role   : $role"
    echo
done

# If the RealSense SDK is already installed, attach the corresponding official information
if command -v rs-enumerate-devices &>/dev/null; then
    echo "=== RealSense SDK detected ==="
    rs-enumerate-devices | grep -E "(Name|video[0-9]+)"
fi
