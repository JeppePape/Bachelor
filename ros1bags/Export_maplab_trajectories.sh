#!/bin/bash

# Source Maplab workspace
source ~/Desktop/maplab/maplab_ws/devel/setup.bash

MAPLAB_OUTPUT_DIR=~/Desktop/maplab_output
SHARED_BAGS_DIR="/mnt/hgfs/Bags/ros1bags"


for map_folder in "$MAPLAB_OUTPUT_DIR"/*_map; do
    [ -d "$map_folder" ] || continue
    BAG_NAME=$(basename "$map_folder" _map)
    BAG_DIR="$SHARED_BAGS_DIR/$BAG_NAME"

    if [ ! -d "$BAG_DIR" ]; then
        echo "⚠️ Corresponding bag folder not found for $BAG_NAME, skipping..."
        continue
    fi

    OUTPUT_FILE="$BAG_DIR/maplab_trajectory.csv"

    if [ -f "$OUTPUT_FILE" ]; then
        echo "✅ Trajectory already exists for $BAG_NAME, skipping..."
        continue
    fi

    echo "📤 Exporting trajectory from $map_folder to $OUTPUT_FILE..."

    echo -e "load --map_folder $map_folder\nexport_trajectory_to_csv --pose_export_file $OUTPUT_FILE --csv_export_format asl\nexit" \
        | rosrun maplab_console maplab_console
done

echo "🏁 All Maplab trajectory exports complete."
