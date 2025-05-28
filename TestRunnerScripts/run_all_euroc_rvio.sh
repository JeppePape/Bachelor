#!/bin/bash

BASE_DIR="/mnt/hgfs/Bags/ros1bags"
LOG_DIR="/mnt/hgfs/Bags/TestRunnerScripts"
PLAYBACK_RATE=1
source ~/Desktop/R-VIO/catkin_ws/devel/setup.bash

# Trap cleanup on interrupt
cleanup() {
    echo "🧹 Cleaning up background processes..."
    [ -n "$RVIZ_PID" ] && kill -INT $RVIZ_PID 2>/dev/null
    [ -n "$RVIO_PID" ] && kill -INT $RVIO_PID 2>/dev/null
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

# ✅ Ensure simulated time is set before running anything
echo "⏰ Setting /use_sim_time to true..."
rosparam set use_sim_time true
sleep 1


for folder in "$BASE_DIR"/*; do
    [ -d "$folder" ] || continue
    BAG_NAME=$(basename "$folder")
    BAG_PATH="$folder/$BAG_NAME.bag"

    if [ ! -f "$BAG_PATH" ]; then
        echo "⚠️ Bag not found: $BAG_PATH, skipping..."
        continue
    fi

    if [ -f "$folder/rvio_trajectory.csv" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "🚀 Launching rvio node..."
    roslaunch rvio euroc.launch &
    RVIO_PID=$!

    echo "🚀 Launching RVIZ..."
    rviz -d ~/Desktop/R-VIO/catkin_ws/src/r-vio/config/rvio_rviz.rviz &
    RVIZ_PID=$!

    sleep 5

    echo "📊 Starting resource logger..."
    bash "$LOG_DIR/log_resources_rvio.sh" &
    LOGGER_PID=$!

    echo "📈 Recording odometry to CSV..."
    rostopic echo -p /rvio/odometry > "$BASE_DIR/rvio_trajectory.csv" &
    RECORD_PID=$!

    echo "🎞️ Playing ROS1 bag: $BAG_PATH"
    rosbag play "$BAG_PATH" --clock --rate $PLAYBACK_RATE  -s 0.5  /cam0/image_raw:=/camera/image_raw /imu0:=/imu &
    BAG_PID=$!

    wait $BAG_PID

    echo "🛑 Bag done, cleaning up..."

    kill -INT $RVIO_PID
    timeout 15s wait $RVIO_PID || kill -9 $RVIO_PID

    kill -INT $RVIZ_PID
    timeout 10s wait $RVIZ_PID || kill -9 $RVIZ_PID

    kill -INT $LOGGER_PID
    timeout 10s wait $LOGGER_PID || kill -9 $LOGGER_PID

    kill -INT $RECORD_PID
    timeout 10s wait $RECORD_PID || kill -9 $RECORD_PID

    echo "📁 Moving output files..."
    [ -f "$BASE_DIR/rvio_trajectory.csv" ] && mv "$BASE_DIR/rvio_trajectory.csv" "$folder/rvio_trajectory.csv"
    [ -f "$LOG_DIR/RVIO_ROS1_log.csv" ] && mv "$LOG_DIR/RVIO_ROS1_log.csv" "$folder/RVIO_ROS1_log.csv"

done

[ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID

echo "🏁 All R-VIO bags processed."
