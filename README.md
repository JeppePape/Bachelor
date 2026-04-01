# Bachelor Thesis – Comparison of Monocular Visual-Inertial Odometry in Aerial Drones

This repository contains the code, scripts, and configuration used for my bachelor thesis at Aarhus University.  
The project compares multiple monocular visual-inertial odometry (VIO) methods on aerial drone datasets, with focus on practical evaluation, performance/resource usage, and repeatable experiment runs.

## Thesis report
- Full thesis report (Markdown): [REPORT.md](bachelor_report_export/REPORT.md)

## Repository contents (high level)
- `bachelor_report_export/` – Markdown version of the thesis report + extracted figures
- `DataConversionScripts/` – scripts for dataset conversion/preprocessing
- `TestRunnerScripts/` – scripts for running/automating experiments
- `config_files/` – configuration files used in experiments
- `orbslam3_ros2/`, `imu_combiner/` – ROS/SLAM related components used during evaluation