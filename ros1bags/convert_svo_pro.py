import sys
import csv

if len(sys.argv) != 3:
    print("Usage: python3 convert_svo_pro.py <input_csv> <output_tum>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    reader = csv.DictReader(infile)
    for row in reader:
        timestamp_ns = int(row['%time'])
        timestamp_s = timestamp_ns / 1e9
        tx = row['field.pose.position.x']
        ty = row['field.pose.position.y']
        tz = row['field.pose.position.z']
        qx = row['field.pose.orientation.x']
        qy = row['field.pose.orientation.y']
        qz = row['field.pose.orientation.z']
        qw = row['field.pose.orientation.w']

        outfile.write(f"{timestamp_s:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
