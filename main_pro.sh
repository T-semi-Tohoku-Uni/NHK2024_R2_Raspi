#!/bin/bash

cd /home/pi/NHK2024/NHK2024_R2_Raspi
. ./env/bin/activate
rm -rf logs
mkdir logs
chmod 777 logs
nohup python -u src/main.py

exit 0