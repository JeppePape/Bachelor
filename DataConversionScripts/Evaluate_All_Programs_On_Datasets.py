import os
import subprocess
import pandas as pd

# ---------- Configurable datasets ----------
datasets = [
    "MH_01_easy","MH_02_easy","MH_03_medium","MH_04_difficult","MH_05_difficult","V1_01_easy","V1_02_medium","V1_03_difficult","V2_01_easy",
    "V2_02_medium","V2_03_difficult"
]

# ---------- Define all trajectory methods and their folder structure ----------
methods = [
    {"name": "VINS-Mono", "folder": "ros1bags", "file": "tum_vinsmono.txt", "log_file": "VINS_MONO_log.csv"},
    {"name": "VINS-Fusion", "folder": "ros1bags", "file": "tum_vinsfusion.txt", "log_file": "VINS_FUSION_log.csv"},
    {"name": "Kimera", "folder": "ros1bags", "file": "tum_kimera.txt", "log_file": "KIMERA_ROS1_log.csv"},
    {"name": "SVO", "folder": "ros1bags", "file": "svo_traj_tum.txt", "log_file": "SVO_ROS1_log.csv"},
    {"name": "Maplab", "folder": "ros1bags", "file": "tum_maplab.txt", "log_file": "MAPLAB_ROS1_log.csv"},
    {"name": "ORB-SLAM3", "folder": "ros2bags", "file": "OrbSlam3TUM.txt", "log_file": "ORBSLAM3_log.csv"},
    {"name": "VI-Stereo-DSO", "folder": "MavEurocFormat", "file": "mav0/nt_{dataset}.txt", "log_file": "VIDSO_ASL_log.csv"},
    {"name": "OKVIS2", "folder": "MavEurocFormat", "file": "mav0/tum_okvis2.txt", "log_file": "OKVIS2_ASL_log.csv"},
    {"name": "RVIO", "folder": "ros1bags", "file": "tum_rvio.txt", "log_file": "RVIO_ROS1_log.csv"},
    {"name": "ORB-SLAM3_2", "folder": "ros2bags", "file": "OrbSlam3TUM_mono2.txt", "log_file": "ORBSLAM3_log_mono2.csv"},
    {"name": "ORB-SLAM3_3", "folder": "ros2bags", "file": "OrbSlam3TUM_mono3.txt", "log_file": "ORBSLAM3_log_mono3.csv"},
]

# ---------- Output CSV ----------
output_csv = r"C:\Users\jeppe\Desktop\Bags\DataConversionScripts\evo_ape_all_results.csv"

# Ensure CSV exists and create header if not
if not os.path.isfile(output_csv):
    df_init = pd.DataFrame(columns=['dataset', 'method', 'trajectory_file', 'mae (m)', 'rmse (m)', 'mean (m)', 'median (m)', 'max (m)', 'min (m)', 'std (m)', 'sse','avg_cpu (%)', 'avg_mem (MB)'])
    df_init.to_csv(output_csv, index=False)

# ---------- Evaluation loop ----------
for dataset_name in datasets:
    # GT path
    gt_file = os.path.join(r"C:\Users\jeppe\Desktop\Bags\MavEurocFormat", dataset_name, "mav0", "state_groundtruth_estimate0", "data.csv")

    for method in methods:
        # Build trajectory path
        if method["folder"] == "MavEurocFormat" and "{dataset}" in method["file"]:
            traj_file = os.path.join(r"C:\Users\jeppe\Desktop\Bags\MavEurocFormat", dataset_name, method["file"].replace("{dataset}", dataset_name))
        else:
            traj_file = os.path.join(r"C:\Users\jeppe\Desktop\Bags", method["folder"], dataset_name, method["file"])

        if not os.path.isfile(traj_file):
            print(f"❌ Skipping missing trajectory: {traj_file}")
            continue

        # Build evo_ape command
        command = [
            'evo_ape','euroc',gt_file,traj_file,'--align','--no_warnings'
        ]

        print(f"➡ Running: {' '.join(command)}")

        try:
            # Run evo_ape and capture output
            output = subprocess.check_output(command, stderr=subprocess.STDOUT, text=True)
            print(output)

            # Parse key metrics
            result = {'dataset': dataset_name, 'method': method["name"], 'trajectory_file': os.path.basename(traj_file)}
            for line in output.splitlines():
                line_clean = line.strip()
                if line_clean.startswith('mae'):
                    result['mae (m)'] = float(line_clean.split()[-1])
                elif line_clean.startswith('max'):
                    result['max (m)'] = float(line_clean.split()[-1])
                elif line_clean.startswith('mean'):
                    result['mean (m)'] = float(line_clean.split()[-1])
                elif line_clean.startswith('median'):
                    result['median (m)'] = float(line_clean.split()[-1])
                elif line_clean.startswith('min'):
                    result['min (m)'] = float(line_clean.split()[-1])
                elif line_clean.startswith('rmse'):
                    result['rmse (m)'] = float(line_clean.split()[-1])
                elif line_clean.startswith('sse'):
                    result['sse'] = float(line_clean.split()[-1])
                elif line_clean.startswith('std'):
                    result['std (m)'] = float(line_clean.split()[-1])
            # ---------- Parse matching log file ----------
            log_file = os.path.join(os.path.dirname(traj_file), method["log_file"])
            if os.path.isfile(log_file):
                try:
                    log_df = pd.read_csv(log_file)

                    # Assuming first column is CPU (%) and second column is Memory (MB)
                    avg_cpu = log_df.iloc[:, 1].mean()
                    avg_mem = log_df.iloc[:, 2].mean()

                    result['avg_cpu (%)'] = round(avg_cpu, 2)
                    result['avg_mem (MB)'] = round(avg_mem, 2)
                except Exception as e:
                    print(f"❌ Error reading log file {log_file}: {e}")
                    result['avg_cpu (%)'] = 'N/A'
                    result['avg_mem (MB)'] = 'N/A'
            else:
                print(f"⚠️ Log file not found: {log_file}")
                result['avg_cpu (%)'] = 'N/A'
                result['avg_mem (MB)'] = 'N/A'        

            # Append to global CSV
            df = pd.read_csv(output_csv)
            df = pd.concat([df, pd.DataFrame([result])], ignore_index=True)
            df.to_csv(output_csv, index=False)
            print(f"✅ Result added to {output_csv}")

        except subprocess.CalledProcessError as e:
            print(f"❌ Error running evo_ape for {traj_file}:\n{e.output}")
