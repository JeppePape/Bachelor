import os
import subprocess

# ----------------- Configuration -------------------
# Dataset and script paths (adjust if needed)
mav_euroc_root = r"C:\Users\jeppe\Desktop\Bags\MavEurocFormat"
ros1bags_root = r"C:\Users\jeppe\Desktop\Bags\ros1bags"
converter_scripts_folder = r"C:\Users\jeppe\Desktop\Bags\DataConversionScripts"

# List of scripts to run per scenario
mav_euroc_converters = ['convert_okvis2_to_tum.py']
ros1bags_converters = ['convert_kimera_tum.py', 'convert_rvio_tum.py', 'convert_vinsmono_tum.py', 'convert_vinsfusion_tum.py', 'convert_maplab_tum.py']

# ----------------- Functions -------------------
def run_converter(script_name, target_folder):
    script_path = os.path.join(converter_scripts_folder, script_name)
    if not os.path.isfile(script_path):
        print(f"❌ Converter script not found: {script_path}")
        return
    print(f"➡ Running {script_name} in {target_folder}")
    subprocess.run(['python', script_path], cwd=target_folder)
    print(f"✅ Finished {script_name} in {target_folder}")

def process_mav_euroc():
    print(f"\n🚀 Processing MavEurocFormat under: {mav_euroc_root}")
    for name in os.listdir(mav_euroc_root):
        dataset_path = os.path.join(mav_euroc_root, name)
        mav0_path = os.path.join(dataset_path, "mav0")
        if os.path.isdir(mav0_path):
            for script in mav_euroc_converters:
                run_converter(script, mav0_path)
        else:
            print(f"⚠ Skipping {dataset_path}, no 'mav0' folder found.")

def process_ros1bags():
    print(f"\n🚀 Processing ros1bags under: {ros1bags_root}")
    for name in os.listdir(ros1bags_root):
        dataset_path = os.path.join(ros1bags_root, name)
        if os.path.isdir(dataset_path):
            for script in ros1bags_converters:
                run_converter(script, dataset_path)
        else:
            print(f"⚠ Skipping {dataset_path}, not a folder.")

# ----------------- Run all -------------------
process_mav_euroc()
process_ros1bags()
print("\n🏁 All conversions done.")
