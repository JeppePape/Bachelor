import csv

input_file = "KIMERA_traj.csv"
output_file = "tum_kimera.txt"


with open(input_file, 'r', newline='') as f_in, open(output_file, 'w') as f_out:
    reader = csv.reader(f_in, delimiter=',')
    for row in reader:
        if len(row) < 8 or row[0].startswith('#') or 'timestamp' in row[0].lower():
            continue
        try:
            # Remove leading apostrophe and any whitespace
            ts_ns = float(row[0].strip().replace("'", ""))
            x = float(row[1].strip())
            y = float(row[2].strip())
            z = float(row[3].strip())
            qw = float(row[4].strip())
            qx = float(row[5].strip())
            qy = float(row[6].strip())
            qz = float(row[7].strip())

            ts_s = ts_ns / 1e9
            f_out.write(f"{ts_s:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
        except (ValueError, IndexError):
            print(f"⚠ Skipping malformed line: {row}")

print(f"✅ Converted and saved to: {output_file}")