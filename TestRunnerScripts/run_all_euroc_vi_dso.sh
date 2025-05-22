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

    # Start resource logger in background
    ./log_resources_vi_dso.sh "$dataset" &
    LOGGER_PID=$!

    # Change to dataset directory and run VI-DSO
    cd "$SHARED_DATASET_DIR"
    "$VI_DSO_PATH/run_euroc.bash" "$dataset"

    # Kill the resource logger
    echo "🧹 Stopping logger for $dataset (PID: $LOGGER_PID)..."
    kill -INT "$LOGGER_PID"
    timeout 10s wait "$LOGGER_PID" || kill -9 "$LOGGER_PID"

    # Move output files
    src_traj_file="$DATA_DIR/nt_${dataset}.txt"
    if [ -f "$src_traj_file" ]; then
        mv "$src_traj_file" "$folder/mav0/"
    fi

    if [ -f "$SHARED_DATASET_DIR/../TestRunnerScripts/$RESOURCE_LOG" ]; then
        mv "$SHARED_DATASET_DIR/../TestRunnerScripts/$RESOURCE_LOG" "$folder/mav0/"
    fi

    echo ">>> Finished $dataset"
done

echo "Batch run complete."
