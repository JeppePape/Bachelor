#!/bin/bash

# Base directory containing each individual ROS 1 bag folder
BASE_DIR="/mnt/hgfs/Bags/audrone"
LOG_DIR="/mnt/hgfs/Bags/TestRunnerScripts"

# Path to VINS-Fusion config YAML
CONFIG_YAML="C:\Users\jeppe\Desktop\Bags\config_files\vins_fusion_audrone.yaml"

# Playback rate
BAG_RATE=0.5

# Source VINS-Fusion environment
source /home/jeppe/Desktop/Vins-Fusion/catkin_ws/devel/setup.bash

# Cleanup trap to kill background processes on script exit
cleanup() {
    echo "🧹 Cleaning up background processes..."
    [ -n "$LOGGER_PID" ] && kill -INT "$LOGGER_PID" 2>/dev/null
    [ -n "$VINS_PID" ] && kill -INT "$VINS_PID" 2>/dev/null
    [ -n "$ROSCORE_PID" ] && kill -INT "$ROSCORE_PID" 2>/dev/null
    exit 1
}
trap cleanup SIGINT SIGTERM

# Start roscore if not running
if ! pgrep -f "roscore" > /dev/null; then
    echo "🔧 Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 5
else
    echo "✅ roscore already running."
    ROSCORE_PID=""
fi

# Loop through all folders
for folder in "$BASE_DIR"/*; do
    [ -d "$folder" ] || continue
    BAG_NAME=$(basename "$folder")
    BAG_PATH="$folder/$BAG_NAME.bag"
    LOG_PATH="$folder/VINS_FUSION_log.csv"

    if [ ! -f "$BAG_PATH" ]; then
        echo "⚠️ Bag file not found: $BAG_PATH, skipping..."
        continue
    fi

    if [ -f "$folder/vins_fusion_trajectory.csv" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "🚀 Launching VINS node for $BAG_NAME..."
    rosrun vins vins_node "$CONFIG_YAML" &
    VINS_PID=$!

    echo "⏳ Waiting 5s for VINS to initialize..."
    sleep 5

    echo "📊 Starting resource logger..."
    bash "$LOG_DIR/log_resources_vins_fusion.sh" &
    LOGGER_PID=$!

    echo "🎞️ Playing ROS1 bag: $BAG_PATH"
    rosbag play "$BAG_PATH" --rate $BAG_RATE -s 4 &
    BAG_PID=$!

    wait $BAG_PID
    echo "🛑 Bag finished, shutting down VINS and logger..."

    kill -INT $VINS_PID
    timeout 15s wait $VINS_PID || kill -9 $VINS_PID

    kill -INT $LOGGER_PID
    timeout 10s wait $LOGGER_PID || kill -9 $LOGGER_PID

    echo "📁 Moving output files to $folder..."
    [ -f "$LOG_DIR/Vins_fusion_ROS1_log.csv" ] && mv "$LOG_DIR/Vins_fusion_ROS1_log.csv" "$folder/VINS_FUSION_log.csv"
    [ -f "$BASE_DIR/vio.csv" ] && mv "$BASE_DIR/vio.csv" "$folder/vins_fusion_trajectory.csv"

done

# Stop roscore if we started it
if [ -n "$ROSCORE_PID" ]; then
    echo "🛑 Stopping roscore..."
    kill -INT $ROSCORE_PID
    wait $ROSCORE_PID 2>/dev/null
fi

echo "🏁 All VINS-Fusion bags processed."
