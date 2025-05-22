#!/bin/bash

# === CONFIG ===
OKVIS_BIN=~/Desktop/okvis2/build/okvis_app_synchronous
CONFIG=~/Desktop/okvis2/config/euroc.yaml
DATASET_ROOT=/mnt/hgfs/Bags/MavEurocFormat
RESOURCE_LOGGER=/mnt/hgfs/Bags/TestRunnerScripts/log_resources_okvis2.sh
LOG_NAME="OKVIS2_ASL_log.csv"
LOG_SOURCE="/mnt/hgfs/Bags/TestRunnerScripts/$LOG_NAME"

# === MAIN LOOP ===
for folder in "$DATASET_ROOT"/*; do
  if [[ -d "$folder/mav0" ]]; then
    LOG_PATH="$folder/mav0/$LOG_NAME"

    # Skip if resource log already exists
    if [[ -f "$LOG_PATH" ]]; then
      echo "⏭️ Skipping $folder — already processed (resource log found)"
      continue
    fi

    echo "=== Running OKVIS2 on: $folder ==="

    # Clean old logs
    rm -f "$folder/mav0/okvis2_output.log"

    # Start resource logger in background
    bash "$(realpath "$RESOURCE_LOGGER")" "$LOG_SOURCE" &
    LOGGER_PID=$!

    # Run OKVIS2 in background and capture output
    "$OKVIS_BIN" "$CONFIG" "$folder/mav0" > "$folder/mav0/okvis2_output.log" 2>&1 &
    OKVIS_PID=$!

    echo "⏳ Waiting for OKVIS2 to finish..."

    # Watch for completion message or crash
    while true; do
      if grep -q "wait for background optimisation to finish" "$folder/mav0/okvis2_output.log"; then
        echo "✅ OKVIS2 completed processing: $folder"
        break
      fi

      if ! kill -0 $OKVIS_PID 2>/dev/null; then
        echo "❌ OKVIS2 crashed or exited early: $folder"
        break
      fi

      sleep 2
    done

    # Stop logger
    kill $LOGGER_PID 2>/dev/null
    wait $LOGGER_PID 2>/dev/null

    # Move the resource log from TestRunnerScripts into current mav0 folder
    if [ -f "$LOG_SOURCE" ]; then
      mv "$LOG_SOURCE" "$LOG_PATH"
      echo "📝 Moved resource log to $LOG_PATH"
    else
      echo "⚠️  Resource log not found: $LOG_SOURCE"
    fi

    echo "✔ Done: $folder"
  else
    echo "Skipping $folder (no mav0/ directory)"
  fi
done
