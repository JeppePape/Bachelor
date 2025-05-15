import os

# ----------------- Config -------------------
root_folder = r"C:\Users\jeppe\Desktop\Bags"
target_files = [
    'tum_vinsmono.txt',
    'tum_vinsfusion.txt',
    'tum_kimera.txt',
    'svo_traj_tum.txt',
    'tum_maplab.txt',
    'OrbSlam3TUM.txt',
    'tum_okvis2.txt',
    'tum_rvio.txt',
    'nt_MH_01_easy.txt',
    'nt_MH_02_easy.txt',
    'nt_MH_03_medium.txt',
    'nt_MH_04_difficult.txt',
    'nt_MH_05_difficult.txt',
]

mh_trim_rules = {
    "MH_01_easy": 25.0,
    "MH_02_easy": 25.0,
    "MH_03_medium": 13.0,
    "MH_04_difficult": 13.0,
    "MH_05_difficult": 13.0
}

# ----------------- Trim function -------------------
def trim_trajectory(input_file, output_file, trim_start_sec):
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
        print(f"⚠️ No valid data found in {input_file}")
        return

    first_timestamp = cleaned_rows[0][0]

    # Apply trimming
    trimmed_lines = [line for ts, line in cleaned_rows if ts >= first_timestamp + trim_start_sec]

    if not trimmed_lines:
        print(f"⚠️ All data removed after trimming {trim_start_sec} s in {input_file}")
        return

    with open(output_file, 'w') as f_out:
        f_out.write('\n'.join(trimmed_lines) + '\n')

    print(f"✅ Trimmed file saved: {output_file}")

# ----------------- Batch processing -------------------
print(f"\n🚀 Scanning and trimming MH datasets under: {root_folder}")

for dirpath, dirnames, filenames in os.walk(root_folder):
    # Only process folders containing MH_01 to MH_05
    dataset_name = next((d for d in mh_trim_rules if d in dirpath), None)
    if not dataset_name:
        continue

    trim_sec = mh_trim_rules[dataset_name]

    for file in filenames:
        if file in target_files:
            file_path = os.path.join(dirpath, file)
            file_trimmed = os.path.join(dirpath, file.replace('.txt', '_trim.txt'))

            print(f"\n➡ Found {file} in {dirpath} (trimming {trim_sec} s)")
            try:
                trim_trajectory(file_path, file_trimmed, trim_sec)
            except Exception as e:
                print(f"❌ Error trimming {file_path}: {e}")

print("\n🏁 All MH dataset files processed and trimmed.")
