
# ----------------- Timestamp Cutoffs -------------------
# These are absolute timestamp values in seconds and the selected trim time which is roughly at or a few seconds before the drone takes off.
#
# For MH_01_easy:
#   Start time: 1403636578.946206122
#   Duration:   187.749443066
#   Trim:       first 45s
#   Cutoff = start + 45 = 1403636578.946206 + 45 = 1403636623.946206
#
# For MH_02_easy:
#   Start time: 1403636856.854208384
#   Trim:       first 40s
#   Cutoff = start + 40.0 = 1403636856.854208 + 40.0 = 1403636896.854208

# For MH_03_medium:
#   Start time: 1403637130.549400753
#   Trim:       first 19s
#   Cutoff = start + 19.0 = 1403637130.549400 + 19.0 = 1403637149.549401

# For MH_04_difficult:
#   Start time: 1403638127.329872528
#   Trim:       first 20s
#   Cutoff = start + 20.0 = 1403638127.329872 + 20.0 = 1403638147.329872

# For MH_05_difficult:
#   Start time: 1403638518.147705994
#   Trim:       first 20s
#   Cutoff = start + 20.0 = 1403638538.147705

# These are calculated based on the amount of still images in the maveuroc datasets, then taking a generous 1 second off of that estimate.
# For V1_01_easy:
#   Start time: 1403715271.716819613
#   Trim:       first 4s
#   Cutoff = start + 4.0 = 1403715275.716820

# For V1_02_medium:
#   Start time: 1403715517.881911832
#   Trim:       first 4s
#   Cutoff = start + 4.0 = 1403715521.881912

# For V1_03_difficult:
#   Start time: 1403715886.593906337
#   Trim:       first 6s
#   Cutoff = start + 6.0 = 1403715892.593906

# For V2_01_easy:
#   Start time: 1413393211.937170803
#   Trim:       first 4s 
#   Cutoff = start + 4.0 = 1413393215.937170

# For V2_02_medium:
#   Start time: 1413393885.655663492
#   Trim:       first 3.5s 
#   Cutoff = start + 3.5 = 1413393889.155663

# For V2_03_difficult:
#   Start time: 1413394881.215652409
#   Trim:       first 5.0s
#   Cutoff = start + 5.0 = 1413394886.215652

import os

# ----------------- Config -------------------
root_folder = r"C:\Users\jeppe\Desktop\Bags"

datasets = [
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
    "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium", "V2_03_difficult"
]

methods = [
    {"name": "VINS-Mono", "folder": "ros1bags", "file": "tum_vinsmono.txt"},
    {"name": "VINS-Fusion", "folder": "ros1bags", "file": "tum_vinsfusion.txt"},
    {"name": "Kimera", "folder": "ros1bags", "file": "tum_kimera.txt"},
    {"name": "SVO", "folder": "ros1bags", "file": "svo_traj_tum.txt"},
    {"name": "Maplab", "folder": "ros1bags", "file": "tum_maplab.txt"},
    {"name": "ORB-SLAM3", "folder": "ros2bags", "file": "OrbSlam3TUM.txt"},
    {"name": "VI-Stereo-DSO", "folder": "MavEurocFormat", "file": "mav0/nt_{dataset}.txt"},
    {"name": "OKVIS2", "folder": "MavEurocFormat", "file": "mav0/tum_okvis2.txt"},
    {"name": "RVIO", "folder": "ros1bags", "file": "tum_rvio.txt"},
    {"name": "ORB-SLAM3_2", "folder": "ros2bags", "file": "OrbSlam3TUM_mono2.txt"},
    {"name": "ORB-SLAM3_3", "folder": "ros2bags", "file": "OrbSlam3TUM_mono3.txt"},
]

# ----------------- Timestamp Cutoffs -------------------
trim_cutoffs = {
    "MH_01_easy": 1403636623.946206,
    "MH_02_easy": 1403636896.854208,
    "MH_03_medium": 1403637149.549401,
    "MH_04_difficult": 1403638147.329872,
    "MH_05_difficult": 1403638538.147705,
    "V1_01_easy": 1403715275.716820,
    "V1_02_medium": 1403715521.881912,
    "V1_03_difficult": 1403715892.593906,
    "V2_01_easy": 1413393215.937170,
    "V2_02_medium": 1413393889.155663,
    "V2_03_difficult": 1413394886.215652
}

# ----------------- Trim Function -------------------
def trim_trajectory(input_file, output_file, cutoff_time):
    try:
        with open(input_file, 'r') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"❌ Failed to read {input_file}: {e}")
        return

    cleaned_rows = []
    for line in lines:
        stripped_line = line.strip()
        if not stripped_line or stripped_line.startswith('#'):
            continue
        try:
            parts = stripped_line.split()
            timestamp = float(parts[0])
            cleaned_rows.append((timestamp, stripped_line))
        except:
            continue

    if not cleaned_rows:
        print(f"⚠️ No valid data in {input_file}")
        return

    trimmed_lines = [line for ts, line in cleaned_rows if ts >= cutoff_time]

    if not trimmed_lines:
        print(f"⚠️ All data removed after cutoff {cutoff_time} in {input_file}")
        return

    with open(output_file, 'w') as f_out:
        f_out.write('\n'.join(trimmed_lines) + '\n')

    print(f"✅ Trimmed file written: {output_file}")

# ----------------- Trimming Execution -------------------
print(f"\n🚀 Trimming all trajectories in: {root_folder}")

for dataset in datasets:
    cutoff = trim_cutoffs.get(dataset)
    if cutoff is None:
        print(f"⚠️ No cutoff defined for {dataset}, skipping...")
        continue

    for method in methods:
        rel_file = method["file"].replace("{dataset}", dataset)
        input_path = os.path.join(root_folder, method["folder"], dataset, rel_file)
        output_path = input_path.replace(".txt", "_trim.txt")

        if not os.path.isfile(input_path):
            print(f"❌ File not found: {input_path}")
            continue

        print(f"\n➡ Trimming {method['name']} for {dataset}")
        print(f"   From: {input_path}")
        print(f"   To:   {output_path}")
        trim_trajectory(input_path, output_path, cutoff)

print("\n🏁 All trajectory files processed and trimmed.")
