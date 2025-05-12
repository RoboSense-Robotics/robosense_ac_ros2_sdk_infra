#!/bin/bash

# Target VID and PID for the USB device
VID_USB="3840"
PID_USB="1010"
VID_RNDIS="3840"
PID_RNDIS="0000"

# Define the udev rule file path
RULE_FILE="/etc/udev/rules.d/99-usb-AC1-permissions.rules"

echo "Creating udev rule for USB device AC1 !"

# Add the udev rule to allow all users read and write access to the USB device
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$VID_USB\", ATTR{idProduct}==\"$PID_USB\", MODE=\"0666\"" | sudo tee $RULE_FILE > /dev/null
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$VID_RNDIS\", ATTR{idProduct}==\"$PID_RNDIS\", MODE=\"0666\"" | sudo tee -a $RULE_FILE > /dev/null

# Reload udev rules and trigger the new rule
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl restart systemd-udevd

echo "USB permission rule applied successfully. You may need to reconnect the USB device."


