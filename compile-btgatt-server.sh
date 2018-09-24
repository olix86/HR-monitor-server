#!/bin/bash
git pull
cp btgatt-server.c ../blue/bluez-5.49/tools/btgatt-server.c
cd ../blue/bluez-5.49/
make
cp tools/btgatt-server ../../HR-monitor-server/ 
cd ../../HR-monitor-server
