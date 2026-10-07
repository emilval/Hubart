#!/bin/bash

PID = $(lsof -t /dev/ttyACM0)

if [ -n "$PID" ]; then
    echo "Killing process $PID"
    kill -9 $PID
else
    echo "No process found using /dev/ttyACM0"
fi