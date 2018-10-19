#!/bin/sh

nohup sudo /home/myk/workspace_szh/videoRobot_run > ./run1.log 2>&1 &
nohup tail -f /home/myk/RF24-master/examples_linux/input.txt | /home/myk/RF24-master/examples_linux/gettingstarted > run2.log 2>&1 &
