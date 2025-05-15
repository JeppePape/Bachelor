#!/bin/bash

BASE_DIR="/mnt/hgfs/Bags/ros1bags"
MAPLAB_OUTPUT="$HOME/Desktop/maplab_output"
TRAJECTORY_TMP="$HOME/Desktop/trajectory.csv"
PLAYBACK_RATE=1
source ~/Desktop/maplab/maplab_ws/devel/setup.bash

# Cleanup trap
cleanup() {
    echo "🧹 Cleaning up background processes..."
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
    OUTPUT_MAP="$MAPLAB_OUTPUT/${BAG_NAME}_map"
    TRAJ_FINAL="$folder/maplab_trajectory.csv"
    LOGFILE="$folder/MAPLAB_ROS1_log.csv"

    if [ ! -f "$BAG_PATH" ]; then
        echo "⚠️ Bag file not found: $BAG_PATH, skipping..."
        continue
    fi

    if [ -f "$TRAJ_FINAL" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "🚀 Running Maplab for $BAG_NAME..."
    rosrun rovioli tutorial_euroc "$OUTPUT_MAP" "$BAG_PATH" &

    echo "📊 Starting resource logger..."
    bash "$BASE_DIR/log_resources_maplab.sh" > "$LOGFILE" &
    LOGGER_PID=$!

    wait

    echo "📤 Exporting trajectory to CSV..."
    echo -e "load --map_folder $OUTPUT_MAP\nexport_trajectory_to_csv --pose_export_file $TRAJECTORY_TMP --csv_export_format asl\nexit" | rosrun maplab_console maplab_console

    if [ -f "$TRAJECTORY_TMP" ]; then
        mv "$TRAJECTORY_TMP" "$TRAJ_FINAL"
        echo "✅ Trajectory saved to $TRAJ_FINAL"
    else
        echo "❌ Failed to export trajectory."
    fi

    echo "📁 Finished processing $BAG_NAME."
    kill -INT $LOGGER_PID
    timeout 10s wait $LOGGER_PID || kill -9 $LOGGER_PID
done

[ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID
echo "🏁 All Maplab bags processed."
