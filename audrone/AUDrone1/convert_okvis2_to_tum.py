import csv

input_file = "okvis2-slam_trajectory.csv"
output_file = "tum_okvis2.txt"

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    reader = csv.reader(f_in, delimiter=',')
    for row in reader:
        # Skip header or comment lines
        if not row or row[0].startswith('#') or row[0].lower().startswith('timestamp'):
            continue
        try:
            ts_ns = float(row[0])
            x = float(row[1])
            y = float(row[2])
            z = float(row[3])
            qx = float(row[4])
            qy = float(row[5])
            qz = float(row[6])
            qw = float(row[7])
            ts_s = ts_ns / 1e9
            # Write in TUM format
            f_out.write(f"{ts_s:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
        except (ValueError, IndexError):
            print(f"⚠ Skipping malformed line: {row}")

print(f"✅ Converted and saved to: {output_file}")
