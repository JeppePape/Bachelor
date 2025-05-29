import os
import subprocess
import pandas as pd

# === Configuration ===
base_dir = os.path.expanduser("~/Desktop/Bags/audrone")
output_csv = "audrone_orbslam3_timeoffset_results.csv"
orbslam3_files = {
    "ORB-SLAM3": "OrbSlam3TUM.txt",
    "ORB-SLAM3_2": "OrbSlam3TUM_mono2.txt",
    "ORB-SLAM3_3": "OrbSlam3TUM_mono3.txt"
}
offsets = [round(i * 0.05, 2) for i in range(-40, 41)]  # -2.0 to 2.0 in 0.05s steps


# === Initialize output DataFrame ===
if not os.path.isfile(output_csv):
    df_init = pd.DataFrame(columns=[
        "dataset", "method", "offset", "mae (m)", "rmse (m)", "mean (m)", "median (m)", "max (m)", "min (m)", "std (m)", "sse",
        "avg_cpu (%)", "avg_mem (MB)"
    ])
    df_init.to_csv(output_csv, index=False)

# === Loop Through AUDrone Datasets ===
for drone_folder in sorted(os.listdir(base_dir)):
    dataset_path = os.path.join(base_dir, drone_folder)
    if not os.path.isdir(dataset_path):
        continue

    gt_path = os.path.join(dataset_path, "vicon_tum_aligned.txt")
    if not os.path.isfile(gt_path):
        print(f"⚠️  Skipping {drone_folder}: Missing GT file {gt_path}")
        continue

    for method, traj_name in orbslam3_files.items():
        traj_path = os.path.join(dataset_path, traj_name)
        if not os.path.isfile(traj_path):
            print(f"⚠️  Skipping {method} in {drone_folder}: Missing {traj_name}")
            continue

        for offset in offsets:
            print(f"⏱️ {drone_folder} | {method} | Offset: {offset}")

            try:
                result = subprocess.run(
                    ["evo_ape", "tum", gt_path, traj_path,
                     "--align", "--t_offset", str(offset), "--no_warnings"],
                    capture_output=True, text=True, check=True
                )

                metrics = {
                    "dataset": drone_folder,
                    "method": method,
                    "offset": offset
                }

                for line in result.stdout.splitlines():
                    line_clean = line.strip()
                    if line_clean.startswith("mae"):
                        metrics["mae (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("rmse"):
                        metrics["rmse (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("mean"):
                        metrics["mean (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("median"):
                        metrics["median (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("max"):
                        metrics["max (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("min"):
                        metrics["min (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("std"):
                        metrics["std (m)"] = float(line_clean.split()[-1])
                    elif line_clean.startswith("sse"):
                        metrics["sse"] = float(line_clean.split()[-1])

                # Look for a matching log file (same folder)
                log_file = os.path.join(dataset_path, f"{method}_log.csv")
                if os.path.isfile(log_file):
                    try:
                        log_df = pd.read_csv(log_file)
                        metrics["avg_cpu (%)"] = round(log_df.iloc[:, 1].mean(), 2)
                        metrics["avg_mem (MB)"] = round(log_df.iloc[:, 2].mean(), 2)
                    except Exception as e:
                        print(f"❌ Error reading log file {log_file}: {e}")
                        metrics["avg_cpu (%)"] = "N/A"
                        metrics["avg_mem (MB)"] = "N/A"
                else:
                    print(f"⚠️  No log file found for {method} in {drone_folder}")
                    metrics["avg_cpu (%)"] = "N/A"
                    metrics["avg_mem (MB)"] = "N/A"

                # Append result
                df = pd.read_csv(output_csv)
                df = pd.concat([df, pd.DataFrame([metrics])], ignore_index=True)
                df.to_csv(output_csv, index=False)
                print("✅ Result added.")

            except subprocess.CalledProcessError as e:
                print(f"❌ evo_ape failed for {drone_folder} | {method} | Offset {offset}")
                print(e.output)
