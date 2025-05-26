#!/bin/bash

BASE_DIR="/mnt/hgfs/Bags/ros1bags"
MAPLAB_OUTPUT="$HOME/Desktop/maplab_output"
TRAJECTORY_TMP="$HOME/Desktop/trajectory.csv"
PLAYBACK_RATE=1

source ~/Desktop/maplab/maplab_ws/devel/setup.bash
export ROVIO_CONFIG_DIR=~/Desktop/maplab/maplab_ws/src/maplab/applications/rovioli/share
rosparam set use_sim_time true

# Cleanup trap
cleanup() {
    echo "🧹 Cleaning up background processes..."
    pkill -f tutorial_euroc_live
    pkill -f rosbag
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

# -----------------------------
# STAGE 1: Run ROVIOLI + rosbag
# -----------------------------
for folder in "$BASE_DIR"/*; do
    [ -d "$folder" ] || continue
    BAG_NAME=$(basename "$folder")
    BAG_PATH="$folder/$BAG_NAME.bag"
    OUTPUT_MAP="$MAPLAB_OUTPUT/${BAG_NAME}_map"
    LOGFILE="$folder/MAPLAB_ROS1_log.csv"

    if [ ! -f "$BAG_PATH" ]; then
        echo "⚠️ Bag file not found: $BAG_PATH, skipping..."
        continue
    fi

    if [ -d "$OUTPUT_MAP" ] && [ "$(ls -A "$OUTPUT_MAP")" ]; then
        echo "✅ Map already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "🚀 Launching Maplab ROVIOLI for $BAG_NAME..."
    setsid rosrun rovioli tutorial_euroc_live "$OUTPUT_MAP" &
    ROVIOLI_PID=$!
    sleep 5

    echo "🎞️ Playing bag at $BAG_PATH..."
    rosbag play "$BAG_PATH" --clock -r $PLAYBACK_RATE &
    BAG_PID=$!

    LOG_TEMP="$BASE_DIR/../TestRunnerScripts/MAPLAB_ROS1_log.csv"
    bash "$BASE_DIR/../TestRunnerScripts/log_resources_maplab.sh" > "$LOG_TEMP" &
    LOGGER_PID=$!

    # Wait for the bag to finish
    wait $BAG_PID
    echo "✅ Bag playback finished."

    echo "🛑 Sending SIGINT to ROVIOLI (group -$ROVIOLI_PID) to trigger map save..."
    kill -INT -- -$ROVIOLI_PID
    sleep 10

    kill -INT $LOGGER_PID
    timeout 10s bash -c "wait $LOGGER_PID" 2>/dev/null || {
        echo "⚠️ Logger did not exit cleanly, force killing..."
        kill -9 $LOGGER_PID
    }

    # Move log file
    if [ -f "$LOG_TEMP" ]; then
        mv "$LOG_TEMP" "$LOGFILE"
        echo "📦 Moved log file to $LOGFILE"
    else
        echo "⚠️ Logger output missing for $BAG_NAME"
    fi

    echo "📁 Finished processing $BAG_NAME."
done

# -----------------------------
# STAGE 2: Export all trajectories
# -----------------------------
echo "📤 Starting trajectory export for all maps..."

for folder in "$BASE_DIR"/*; do
    [ -d "$folder" ] || continue
    BAG_NAME=$(basename "$folder")
    OUTPUT_MAP="$MAPLAB_OUTPUT/${BAG_NAME}_map"
    TRAJ_FINAL="$folder/maplab_trajectory.csv"

    if [ -f "$TRAJ_FINAL" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping export..."
        continue
    fi

    if [ -d "$OUTPUT_MAP" ] && [ "$(ls -A "$OUTPUT_MAP")" ]; then
        echo "📤 Exporting trajectory for $BAG_NAME..."
        echo -e "load --map_folder $OUTPUT_MAP\nexport_trajectory_to_csv --pose_export_file $TRAJECTORY_TMP --csv_export_format asl\nexit" | rosrun maplab_console maplab_console

        if [ -f "$TRAJECTORY_TMP" ]; then
            mv "$TRAJECTORY_TMP" "$TRAJ_FINAL"
            echo "✅ Trajectory saved to $TRAJ_FINAL"
        else
            echo "❌ Failed to export trajectory for $BAG_NAME"
        fi
    else
        echo "❌ No map found for $BAG_NAME, skipping export."
    fi
done

[ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID
echo "🏁 All Maplab bags processed and trajectories exported."
