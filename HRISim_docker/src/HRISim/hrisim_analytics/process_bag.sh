#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: ./process_bag.sh /path/to/your/bagfile.bag [subject_id]"
    echo "Defaults: subject_id=0 (Human)"
    exit 1
fi

BAG_FILE=$1
BAG_NAME=$(basename "$BAG_FILE" .bag)
SUBJECT_ID=${2:-0}

echo "Processing bag: $BAG_NAME (Subject: $SUBJECT_ID)"

# Start roscore in background if not running
roscore &
CORE_PID=$!
sleep 5

# Set sim time
rosparam set use_sim_time true

# Start the extractor in background
roslaunch hrisim_analytics extract.launch bag_name:="$BAG_NAME" subject:="$SUBJECT_ID" &
EXTRACTOR_PID=$!
sleep 2

# Play the bag
echo "Playing bag file..."
rosbag play "$BAG_FILE" --clock

# Once finished, kill the processes to trigger the save_data shutdown callback
echo "Bag finished. Saving CSV..."
kill -INT $EXTRACTOR_PID
sleep 5
kill -INT $CORE_PID

echo "Done. Check HRISim_docker/src/HRISim/hrisim_analytics/data/$BAG_NAME.csv"
