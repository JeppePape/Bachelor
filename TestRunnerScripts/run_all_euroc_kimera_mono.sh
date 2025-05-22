#!/bin/bash

source ~/Desktop/Bachelor/KIMERA/catkin_ws/devel/setup.bash
roscore &
sleep 5

LOG_SCRIPT="/mnt/hgfs/Bags/TestRunnerScripts/log_resources_KIMERA.sh"
OUTPUT_DIR="/home/jeppe/Desktop/Bachelor/KIMERA/catkin_ws/src/Kimera-VIO-ROS/output_logs/EurocMono"

for folder in /mnt/hgfs/Bags/ros1bags/*; do
    if [[ -d "$folder" ]]; then
        bagfile=$(find "$folder" -maxdepth 1 -name "*.bag" | head -n 1)
        if [[ ! -f "$bagfile" ]]; then
            echo "✘ No bag found in $folder, skipping."
            continue
        fi

        if [[ -f "$folder/KIMERA_traj.csv" && -f "$folder/KIMERA_ROS1_log.csv" ]]; then
            echo "✔ Skipping $folder (already processed)"
            continue
        fi

        echo "=== Running KIMERA-VIO Mono on: $folder ==="

        attempt=1
        while true; do
            echo "▶ Attempt $attempt"

            # Launch Kimera-VIO
            roslaunch kimera_vio_ros kimera_vio_ros_euroc_mono.launch &
            vio_pid=$!
            sleep 5

            # Start resource logger
            bash "$LOG_SCRIPT" "$folder/KIMERA_ROS1_log.csv" &
            logger_pid=$!

            # Play rosbag in background
            rosbag play --clock --rate=0.5 -s 13 "$bagfile" &
            bag_pid=$!

            # Wait for rosbag to finish
            wait $bag_pid

            # Shut down Kimera
            echo "✔ Bag playback ended. Shutting down Kimera..."
            rosnode kill /kimera_vio_ros/kimera_vio_ros_node 2>/dev/null
            rosnode kill /kimera_vio_ros/posegraph_viewer 2>/dev/null
            kill $vio_pid 2>/dev/null
            wait $vio_pid 2>/dev/null

            # Stop logger
            kill $logger_pid 2>/dev/null
            wait $logger_pid 2>/dev/null

            # Check if output was created
            if [[ -f "$OUTPUT_DIR/traj_vio.csv" ]]; then
                mv "$OUTPUT_DIR/traj_vio.csv" "$folder/KIMERA_traj.csv"
                mv "/mnt/hgfs/Bags/TestRunnerScripts/KIMERA_ROS1_log.csv" "$folder/KIMERA_ROS1_log.csv" 2>/dev/null
                echo "✔ Done: $folder"
                break
            else
                echo "✘ Failed attempt $attempt in $folder. Retrying..."
                ((attempt++))
                sleep 2
            fi
        done
    fi
done
