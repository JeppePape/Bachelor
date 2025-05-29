import pandas as pd

# Config
hz = 100.0
bag_start_time = 418.003649217  # replace with your actual SLAM trajectory start time

# Load CSV
df = pd.read_csv("groundtruth/px4test1.csv", skiprows=3)

# Convert relevant columns
cols = ["TX", "TY", "TZ", "RX", "RY", "RZ", "RW"]
for col in cols:
    df[col] = pd.to_numeric(df[col], errors="coerce")
df.dropna(inplace=True)

# New timestamps aligned to SLAM
df["timestamp"] = bag_start_time + (df["Frame"] - 1) / hz

# mm → meters
df["tx"] = df["TX"] / 1000.0
df["ty"] = df["TY"] / 1000.0
df["tz"] = df["TZ"] / 1000.0

# Quaternions reordered
df["qx"] = df["RX"]
df["qy"] = df["RY"]
df["qz"] = df["RZ"]
df["qw"] = df["RW"]

# Output
tum_df = df[["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]]
tum_df.to_csv("vicon_tum_aligned.txt", sep=" ", index=False, header=False)
print("✅ vicon_tum_aligned.txt saved.")
