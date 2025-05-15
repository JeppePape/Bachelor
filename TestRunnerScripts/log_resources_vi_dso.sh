#!/bin/bash

LOGFILE="VIDSO_ASL_log.csv"
echo "timestamp,cpu_percent,ram_percent" > "$LOGFILE"

while true; do
  timestamp=$(date +%s)
  cpu=$(top -bn1 | grep '%Cpu' | awk '{print 100 - $8}')
  ram=$(free | grep Mem | awk '{print $3/$2 * 100.0}')
  echo "$timestamp,$cpu,$ram" >> "$LOGFILE"
  sleep 1
done
