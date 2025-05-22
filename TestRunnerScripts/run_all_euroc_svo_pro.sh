#!/bin/bash
LOG_DIR="/mnt/hgfs/Bags/TestRunnerScripts"
BASE_DIR="/mnt/hgfs/Bags/ros1bags"
PLAYBACK_RATE=1
source ~/Desktop/svo_pro/svo_ws/devel/setup.bash

# Trap cleanup on interrupt
cleanup() {
    echo "🧹 Cleaning up background processes..."
    [ -n "$SVO_PID" ] && kill -INT $SVO_PID 2>/dev/null
    [ -n "$LOGGER_PID" ] && kill -INT $LOGGER_PID 2>/dev/null
    [ -n "$RECORD_PID" ] && kill -INT $RECORD_PID 2>/dev/null
    [ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID 2>/dev/null
    exit 1
}
trap cleanup SIGINT SIGTERM

# Start roscore if needed
if ! pgrep -f "roscore" > /dev/null; then
    echo "🔧 Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 5
fi

for folder in "$BASE_DIR"/*; do
    [ -d "$folder" ] || continue
    BAG_NAME=$(basename "$folder")
    BAG_PATH="$folder/$BAG_NAME.bag"

    if [ ! -f "$BAG_PATH" ]; then
        echo "⚠️ Bag not found: $BAG_PATH, skipping..."
        continue
    fi

    if [ -f "$folder/svo_traj_tum.txt" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "🚀 Launching svo_ros node..."
    roslaunch svo_ros euroc_vio_mono.launch &
    SVO_PID=$!

    sleep 5

    echo "📊 Starting resource logger..."
    bash "$LOG_DIR/log_resources_svo_pro.sh" &
    LOGGER_PID=$!

    echo "📈 Recording trajectory from /svo/pose_cam/0..."
    rostopic echo -p /svo/pose_cam/0 > "$BASE_DIR/svo_trajectory.csv" &
    RECORD_PID=$!

    echo "🎞️ Playing ROS bag: $BAG_PATH"
    rosbag play "$BAG_PATH" --rate $PLAYBACK_RATE -s 15 &
    BAG_PID=$!

    wait $BAG_PID

    echo "🛑 Bag done, cleaning up..."

    kill -INT $SVO_PID
    timeout 15s wait $SVO_PID || kill -9 $SVO_PID

    kill -INT $LOGGER_PID
    timeout 10s wait $LOGGER_PID || kill -9 $LOGGER_PID

    kill -INT $RECORD_PID
    timeout 10s wait $RECORD_PID || kill -9 $RECORD_PID

    echo "🧪 Converting to TUM format..."
    python3 "$LOG_DIR/convert_svo_pro.py" "$BASE_DIR/svo_trajectory.csv" "$folder/svo_traj_tum.txt"


    echo "📁 Moving log file..."
    [ -f "$LOG_DIR/SVO_PRO_ROS1_log.csv" ] && mv "$LOG_DIR/SVO_PRO_ROS1_log.csv" "$folder/SVO_ROS1_log.csv"

done

[ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID

echo "🏁 All EuRoC bags processed with SVO-Pro."
