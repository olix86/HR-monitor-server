#!/bin/bash
sudo btmgmt -i hci0 power off
sudo btmgmt -i hci0 le on
sudo btmgmt -i hci0 connectable on
sudo btmgmt -i hci0 name "Heart Rate Server"
sudo btmgmt -i hci0 advertising on
sudo btmgmt -i hci0 power on
./btgatt-server -i hci0 -s low -t public -r -v
