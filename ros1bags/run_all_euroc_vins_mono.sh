#!/bin/bash

BASE_DIR="/mnt/hgfs/Bags/ros1bags"
PLAYBACK_RATE=0.5
source ~/Desktop/Vins-Mono/catkin_ws/devel/setup.bash

# Trap cleanup on interrupt
cleanup() {
    echo "🧹 Cleaning up background processes..."
    [ -n "$RVIZ_PID" ] && kill -INT $RVIZ_PID 2>/dev/null
    [ -n "$VINS_PID" ] && kill -INT $VINS_PID 2>/dev/null
    [ -n "$LOGGER_PID" ] && kill -INT $LOGGER_PID 2>/dev/null
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

    if [ -f "$folder/vins_mono_trajectory.csv" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "🚀 Launching vins_estimator..."
    roslaunch vins_estimator euroc.launch &
    VINS_PID=$!

    echo "🚀 Launching VINS RViz..."
    roslaunch vins_estimator vins_rviz.launch &
    RVIZ_PID=$!

    sleep 5

    echo "📊 Starting resource logger..."
    bash "$BASE_DIR/log_resources_vins_mono.sh" &
    LOGGER_PID=$!

    echo "🎞️ Playing ROS1 bag: $BAG_PATH"
    rosbag play "$BAG_PATH" --rate $PLAYBACK_RATE &
    BAG_PID=$!

    wait $BAG_PID

    echo "🛑 Bag done, cleaning up..."

    kill -INT $VINS_PID
    timeout 15s wait $VINS_PID || kill -9 $VINS_PID

    kill -INT $RVIZ_PID
    timeout 10s wait $RVIZ_PID || kill -9 $RVIZ_PID

    kill -INT $LOGGER_PID
    timeout 10s wait $LOGGER_PID || kill -9 $LOGGER_PID

    echo "📁 Moving output files..."
    [ -f "$BASE_DIR/Vins_mono_ROS1_log.csv" ] && mv "$BASE_DIR/Vins_mono_ROS1_log.csv" "$folder/VINS_MONO_log.csv"
    [ -f "$BASE_DIR/vins_result_no_loop.csv" ] && mv "$BASE_DIR/vins_result_no_loop.csv" "$folder/vins_mono_trajectory.csv"

done

[ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID

echo "🏁 All VINS-Mono bags processed."
