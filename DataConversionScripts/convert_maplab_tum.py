import csv

input_file = "maplab_trajectory.csv"
output_file = "tum_maplab.txt"

with open(input_file, 'r', newline='') as f_in, open(output_file, 'w') as f_out:
    reader = csv.reader(f_in, delimiter=',')
    line_num = 0
    for row in reader:
        line_num += 1
        if row[0].startswith('#') or 'timestamp' in row[0].lower():
            continue  # skip header or comments
        try:
            ts_ns = float(row[0].strip())
            x = float(row[3].strip())
            y = float(row[4].strip())
            z = float(row[5].strip())
            qw = float(row[6].strip())
            qx = float(row[7].strip())
            qy = float(row[8].strip())
            qz = float(row[9].strip())

            ts_s = ts_ns / 1e9
            # Write in TUM format (timestamp x y z qx qy qz qw)
            f_out.write(f"{ts_s:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
        except (ValueError, IndexError) as e:
            print(f"⚠ Skipping malformed line {line_num}: {row} ({e})")

print(f"✅ Converted and saved to: {output_file}")
