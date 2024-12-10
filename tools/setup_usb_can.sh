#!/bin/bash

# Exit on error
set -e

# Must run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root"
    exit 1
fi

# Get the actual user who called sudo
REAL_USER=${SUDO_USER:-$(whoami)}
if [ "$REAL_USER" = "root" ]; then
    echo "Please run with sudo instead of as root directly"
    exit 1
fi

echo "Setting up ZLG USBCAN permissions for user: $REAL_USER"

# Create canbus group if it doesn't exist
if ! getent group canbus >/dev/null; then
    echo "Creating canbus group..."
    groupadd --system canbus
    echo "Created canbus group"
else
    echo "canbus group already exists"
fi

# Add user to canbus group if not already a member
if ! groups "$REAL_USER" | grep -q "\bcanbus\b"; then
    echo "Adding user $REAL_USER to canbus group..."
    usermod -a -G canbus "$REAL_USER"
    echo "Added $REAL_USER to canbus group"
else
    echo "User $REAL_USER is already in canbus group"
fi

# Create/update udev rule
UDEV_RULE_FILE="/etc/udev/rules.d/99-zlg-can.rules"

# Detect the CAN device model
CAN_DEVICE_MODEL=""
if lsusb | grep -q "USBCANFD-100U"; then
    CAN_DEVICE_MODEL="USBCANFD-100U"
    echo "Detected ZLG USBCAN device model: USBCANFD-100U"
elif lsusb | grep -q "USBCANFD-200U"; then
    CAN_DEVICE_MODEL="USBCANFD-200U"
    echo "Detected ZLG USBCAN device model: USBCANFD-200U"
else
    echo "WARNING: No ZLG USBCAN device currently connected"
    echo "Permissions will be applied when device is plugged in"
    exit 0
fi

if [ ! -f "$UDEV_RULE_FILE" ] || ! grep -q "idVendor.*3068" "$UDEV_RULE_FILE"; then
    echo "Creating udev rule..."
    echo "$UDEV_RULE" > "$UDEV_RULE_FILE"
    echo "Created udev rule: $UDEV_RULE_FILE"
else
    echo "udev rule already exists"
fi

# Reload udev rules
echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger
echo "Reloaded udev rules"

# Verify device presence and permissions
echo "Verifying device permissions..."
DEVICE_FOUND=false

for device in $(find /dev/bus/usb -mindepth 2 -maxdepth 2 -type c); do
    # Store udevadm output in a variable to avoid running it multiple times
    UDEVADM_INFO=$(udevadm info -a -n "$device" 2>/dev/null) || continue

    # Use simpler grep patterns without curly braces
    if echo "$UDEVADM_INFO" | grep -q "idVendor.*3068" && \
       echo "$UDEVADM_INFO" | grep -q "idProduct.*0009"; then
        DEVICE_FOUND=true

        # Check permissions
        perms=$(stat -c "%a" "$device")
        group=$(stat -c "%G" "$device")

        echo "Found ZLG USBCAN device: $device"
        echo "Current permissions: $perms"
        echo "Current group: $group"

        if [ "$perms" = "660" ] && [ "$group" = "canbus" ]; then
            echo "Device permissions are correctly configured"
        else
            echo "WARNING: Device permissions or group are not as expected"
            echo "Expected: 660 canbus"
            echo "Got: $perms $group"
            echo "You may need to unplug and replug the device"
        fi
        break
    fi
done

if [ "$DEVICE_FOUND" = false ]; then
    echo "WARNING: No ZLG USBCAN device currently connected"
    echo "Permissions will be applied when device is plugged in"
fi

echo
echo "Setup completed!"
if [ "$DEVICE_FOUND" = false ] || [ "$perms" != "660" ] || [ "$group" != "canbus" ]; then
    echo "Please unplug and replug your ZLG USBCAN device"
    echo "Then log out and log back in for group changes to take effect"
fi
