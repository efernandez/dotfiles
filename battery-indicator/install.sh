#!/bin/bash

# http://www.everythingisvoid.com/uncategorized/simple-battery-status-indicator-awesome-window-manager#sthash.rLKCN08B.dpbs
sudo apt-get install -y lua5.1 luarocks libgirepository1.0-dev acpi

sudo luarocks install battery_status

