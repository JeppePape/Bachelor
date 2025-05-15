#!/bin/bash

echo "Starting VI-DSO batch run..."

VI_DSO_PATH="/home/jeppe/Desktop/Bachelor/VI-Stereo-DSO"
SHARED_DATASET_DIR="/mnt/hgfs/Bags/MavEurocFormat"
RESOURCE_LOG="VIDSO_ASL_log.csv"
DATA_DIR="${SHARED_DATASET_DIR}/data"

mkdir -p "$DATA_DIR"

for folder in "$SHARED_DATASET_DIR"/*; do
    dataset=$(basename "$folder")

    # Skip non-directories or 'data'
    if [ ! -d "$folder" ] || [ "$dataset" = "data" ]; then
        continue
    fi

    traj_file="$folder/mav0/nt_${dataset}.txt"
    if [ -f "$traj_file" ]; then
        echo ">>> Skipping $dataset (trajectory already exists)"
        continue
    fi

    echo ">>> Running VI-DSO on dataset: $dataset"

    # Start resource logger
    ./log_resources_vi_dso.sh "$dataset" &

    # Launch VI-DSO (will have to be manually stopped when done)
    "$VI_DSO_PATH/run_euroc.bash" "$dataset"

    # Move the output files if they were created
    src_traj_file="$DATA_DIR/nt_${dataset}.txt"
    if [ -f "$src_traj_file" ]; then
        mv "$src_traj_file" "$folder/mav0/"
    fi

    if [ -f "$SHARED_DATASET_DIR/$RESOURCE_LOG" ]; then
        mv "$SHARED_DATASET_DIR/$RESOURCE_LOG" "$folder/mav0/"
    fi

    echo ">>> Finished $dataset"
done

echo "Batch run complete."
