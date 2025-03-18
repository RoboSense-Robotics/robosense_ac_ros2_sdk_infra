#!/bin/bash

# Target VID and PID for the USB device
VID="3840"
PID="1010"

# Define the udev rule file path
RULE_FILE="/etc/udev/rules.d/99-usb-metaS-permissions.rules"

echo "Creating udev rule for USB device metaS!"

# Add the udev rule to allow all users read and write access to the USB device
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$VID\", ATTR{idProduct}==\"$PID\", MODE=\"0666\"" | sudo tee $RULE_FILE > /dev/null

# Reload udev rules and trigger the new rule
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl restart systemd-udevd

echo "USB permission rule applied successfully . You may need to reconnect the USB device."


