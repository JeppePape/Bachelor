#!/bin/bash

# Path to all your bag folders
BASE_DIR="/mnt/hgfs/Bags/Ros2Bags"
LOG_DIR="/mnt/hgfs/Bags/TestRunnerScripts"

# Path to vocabulary and settings
VOCAB="$HOME/Desktop/Bachelor/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt"
YAML="$HOME/Desktop/Bachelor/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml"

# SLAM executable
SLAM_CMD="ros2 run orbslam3 mono-inertial $VOCAB $YAML --ros-args --remap camera:=/cam0/image_raw --remap imu:=/imu0"

# Playback rate
BAG_RATE=0.5

for folder in "$BASE_DIR"/*/; do
    echo "🔁 Processing folder: $folder"
    BAG_NAME=$(basename "$folder")
    BAG_PATH="$folder/$BAG_NAME.db3"
    LOG_PATH="$folder/ORBSLAM3_log.csv"

    if [ -f "$folder/OrbSlam3TUM.txt" ]; then
        echo "⚡ Trajectory already exists, skipping $BAG_NAME."
        continue
    fi

    if [ ! -f "$BAG_PATH" ]; then
        echo "⚠️ Bag not found: $BAG_PATH, skipping..."
        continue
    fi

    retry_count=0
    while true; do
        ((retry_count++))
        echo "🚀 Launching ORB-SLAM3..."
        $SLAM_CMD &
        SLAM_PID=$!

        echo "⏳ Waiting 5 seconds for ORB-SLAM3 to initialize..."
        sleep 5

        echo "📊 Starting resource logger..."
        bash "$LOG_DIR/log_resources_ORBSLAM3.sh" > "ORBSLAM3_log.csv" &
        LOGGER_PID=$!

        echo "🎞️ Playing ROS2 bag: $BAG_PATH"
        ros2 bag play "$BAG_PATH" --rate $BAG_RATE --start-offset 5 &
        BAG_PID=$!

        # Monitor bag and ORB-SLAM3 processes
        while kill -0 $BAG_PID 2>/dev/null; do
            if ! kill -0 $SLAM_PID 2>/dev/null; then
                echo "❌ ORB-SLAM3 crashed during playback. Retrying..."
                kill $BAG_PID $LOGGER_PID 2>/dev/null
                wait $BAG_PID $LOGGER_PID 2>/dev/null
                sleep 2
                continue 2
            fi
            sleep 1
        done

        # Bag finished
        if kill -0 $SLAM_PID 2>/dev/null; then
            echo "✅ Bag completed successfully for $BAG_NAME after $retry_count attempt(s)."
            echo "🛑 Gracefully shutting down ORB-SLAM3 and logger..."
            ps --ppid $SLAM_PID -o pid= | xargs -r kill -INT
            timeout 15s wait $SLAM_PID 2>/dev/null || kill -9 $SLAM_PID
            sleep 5
            ps --ppid $LOGGER_PID -o pid= | xargs -r kill -INT
            timeout 15s wait $LOGGER_PID 2>/dev/null || kill -9 $LOGGER_PID
            sleep 2
            break
        else
            echo "❌ ORB-SLAM3 crashed after playback. Retrying..."
            kill $LOGGER_PID
            wait $LOGGER_PID 2>/dev/null
            sleep 2
        fi
    done

# Move result files to the folder
for f in OrbSlam3TUM.txt OrbSlam3EuRoC.txt OrbSlam3KITTI.txt; do
    if [ -f "$f" ]; then
        mv "$f" "$folder/"
    fi
done

# Move ORBSLAM3_log.csv from LOG_DIR to the folder
if [ -f "$LOG_DIR/ORBSLAM3_log.csv" ]; then
    mv "$LOG_DIR/ORBSLAM3_log.csv" "$folder/"
fi
done

echo "🏁 All bags processed."
