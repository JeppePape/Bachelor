import csv

input_file = "rvio_trajectory.csv"
output_file = "tum_rvio.txt"

with open(input_file, 'r', newline='') as f_in, open(output_file, 'w') as f_out:
    reader = csv.reader(f_in, delimiter=',')
    line_num = 0
    for row in reader:
        line_num += 1
        if row[0].startswith('%') or row[0].startswith('#') or 'stamp' in row[0].lower():
            continue  # skip header or comments
        try:
            ts_ns = float(row[2].strip())  # field.header.stamp
            x = float(row[5].strip())      # position.x
            y = float(row[6].strip())      # position.y
            z = float(row[7].strip())      # position.z
            qx = float(row[8].strip())     # orientation.x
            qy = float(row[9].strip())     # orientation.y
            qz = float(row[10].strip())    # orientation.z
            qw = float(row[11].strip())    # orientation.w

            ts_s = ts_ns / 1e9
            f_out.write(f"{ts_s:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
        except (ValueError, IndexError) as e:
            print(f"⚠ Skipping malformed line {line_num}: {row} ({e})")

print(f"✅ Converted and saved to: {output_file}")
