#!/bin/bash

if [ "$#" -lt 1 ]; then
    echo "Usage: ./process_bag.sh /path/to/your/bagfile.bag [subject_id] [max_duration]"
    echo "Defaults: subject_id=0 (Human), max_duration=330 (seconds)"
    exit 1
fi

BAG_FILE=$1
BAG_NAME=$(basename "$BAG_FILE" .bag)
SUBJECT_ID=${2:-0}
MAX_DURATION=${3:-330}

# Re-source the workspace: a shell opened before the last catkin build has a
# stale ROS_PACKAGE_PATH and roslaunch would not resolve this package
source "$HOME/ros_ws/devel/setup.bash"

echo "Processing bag: $BAG_NAME (Subject: $SUBJECT_ID, Max duration: ${MAX_DURATION}s)"

# Start roscore in background if not running
roscore &
CORE_PID=$!
sleep 5

# Set sim time
rosparam set use_sim_time true

# Start the extractor in background
roslaunch hrisim_analytics extract.launch bag_name:="$BAG_NAME" subject:="$SUBJECT_ID" max_duration:="$MAX_DURATION" &
EXTRACTOR_PID=$!
sleep 2

if ! kill -0 $EXTRACTOR_PID 2>/dev/null; then
    echo "ERROR: metrics extractor failed to start. Aborting."
    kill -INT $CORE_PID
    exit 1
fi

# Play the bag
echo "Playing bag file..."
rosbag play "$BAG_FILE" --clock

# Once finished, kill the processes to trigger the save_data shutdown callback
echo "Bag finished. Saving CSV..."
kill -INT $EXTRACTOR_PID
sleep 5
kill -INT $CORE_PID

echo "Done. Check HRISim_docker/src/HRISim/hrisim_analytics/data/$BAG_NAME.csv"
