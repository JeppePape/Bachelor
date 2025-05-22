import os
import pandas as pd
import shutil
import argparse

def convert_to_euroc(sequence_dir_path):
    """
    Converts data from PX4 drone VOXL-mpa format to EuRoC MAV format
    within the given sequence directory.

    Args:
        sequence_dir_path (str): The full path to the sequence folder 
                                 (e.g., '/path/to/your/data/my_sequence_01').
    """
    if not os.path.isdir(sequence_dir_path):
        print(f"Error: Sequence directory not found: {sequence_dir_path}")
        return

    print(f"Processing sequence: {sequence_dir_path}")

    # --- Path definitions ---
    input_tracking_dir = os.path.join(sequence_dir_path, 'tracking')
    input_imu_apps_dir = os.path.join(sequence_dir_path, 'imu_apps')
    input_tracking_csv_path = os.path.join(input_tracking_dir, 'data.csv')
    input_imu_csv_path = os.path.join(input_imu_apps_dir, 'data.csv')

    mav0_dir = os.path.join(sequence_dir_path, 'mav0')
    cam0_dir = os.path.join(mav0_dir, 'cam0')
    cam0_data_dir = os.path.join(cam0_dir, 'data')
    imu0_dir = os.path.join(mav0_dir, 'imu0')

    output_cam0_csv_path = os.path.join(cam0_dir, 'data.csv')
    output_imu0_csv_path = os.path.join(imu0_dir, 'data.csv')

    # --- Create EuRoC MAV directory structure ---
    print("Creating EuRoC MAV directory structure...")
    os.makedirs(cam0_data_dir, exist_ok=True)
    os.makedirs(imu0_dir, exist_ok=True)
    print(f"  Ensured directory exists: {cam0_data_dir}")
    print(f"  Ensured directory exists: {imu0_dir}")

    # --- Process IMU data ---
    print("Processing IMU data...")
    if os.path.exists(input_imu_csv_path):
        try:
            imu_df = pd.read_csv(input_imu_csv_path)
            
            required_imu_cols = ['timestamp(ns)', 'GX(rad/s)', 'GY(rad/s)', 'GZ(rad/s)', 
                                 'AX(m/s2)', 'AY(m/s2)', 'AZ(m/s2)']
            missing_imu_cols = [col for col in required_imu_cols if col not in imu_df.columns]

            if missing_imu_cols:
                print(f"  Error: Missing required columns in IMU CSV ({input_imu_csv_path}): {missing_imu_cols}")
            else:
                euroc_imu_df = pd.DataFrame({
                    '#timestamp[ns]': imu_df['timestamp(ns)'],
                    'w_RS_S_x[rad s^-1]': imu_df['GX(rad/s)'],
                    'w_RS_S_y[rad s^-1]': imu_df['GY(rad/s)'],
                    'w_RS_S_z[rad s^-1]': imu_df['GZ(rad/s)'],
                    'a_RS_S_x[m s^-2]': imu_df['AX(m/s2)'],
                    'a_RS_S_y[m s^-2]': imu_df['AY(m/s2)'],
                    'a_RS_S_z[m s^-2]': imu_df['AZ(m/s2)']
                })
                
                euroc_imu_df.to_csv(output_imu0_csv_path, index=False, header=True)
                print(f"  IMU data converted and saved to: {output_imu0_csv_path}")
        except pd.errors.EmptyDataError:
            print(f"  Error: IMU data CSV file {input_imu_csv_path} is empty.")
        except Exception as e:
            print(f"  Error processing IMU data from {input_imu_csv_path}: {e}")
    else:
        print(f"  IMU data file not found: {input_imu_csv_path}")

    # --- Process Camera data and images ---
    print("Processing Camera data and images...")
    if os.path.exists(input_tracking_csv_path):
        try:
            cam_df = pd.read_csv(input_tracking_csv_path)
            
            required_cam_cols = ['timestamp(ns)']
            missing_cam_cols = [col for col in required_cam_cols if col not in cam_df.columns]

            if missing_cam_cols:
                 print(f"  Error: Missing required columns in Camera CSV ({input_tracking_csv_path}): {missing_cam_cols}")
            else:
                new_cam_data_list = []
                image_files_to_move_and_rename = []

                print(f"  Found {len(cam_df)} entries in camera CSV: {input_tracking_csv_path}")

                for idx, row in cam_df.iterrows():
                    try:
                        # Ensure timestamp is integer for filename and CSV consistency
                        timestamp_ns = int(row['timestamp(ns)'])
                    except ValueError:
                        print(f"  Warning: Invalid timestamp '{row['timestamp(ns)']}' at data row {idx} in {input_tracking_csv_path}. Skipping this entry.")
                        continue
                    
                    # Original image filename based on 0-indexed row number from CSV
                    # e.g., 0th data row in CSV corresponds to 00000.png
                    original_image_name = f"{idx:05d}.png"
                    new_image_filename = f"{timestamp_ns}.png"
                    
                    source_image_path = os.path.join(input_tracking_dir, original_image_name)
                    destination_image_path = os.path.join(cam0_data_dir, new_image_filename)
                    
                    if os.path.exists(source_image_path):
                        image_files_to_move_and_rename.append((source_image_path, destination_image_path))
                        new_cam_data_list.append({
                            '#timestamp[ns]': timestamp_ns,
                            'filename': new_image_filename
                        })
                    else:
                        print(f"  Warning: Source image {original_image_name} (expected at {source_image_path}) not found. Skipping this camera entry for timestamp {timestamp_ns}.")
                
                # Move/rename images
                moved_count = 0
                print(f"  Attempting to move/rename {len(image_files_to_move_and_rename)} image files...")
                for src, dst in image_files_to_move_and_rename:
                    try:
                        shutil.move(src, dst)
                        moved_count += 1
                    except Exception as e:
                        print(f"  Error moving {src} to {dst}: {e}")
                print(f"  Moved and renamed {moved_count} image files to: {cam0_data_dir}")
                
                # Create and save the new camera CSV
                if new_cam_data_list:
                    euroc_cam_df = pd.DataFrame(new_cam_data_list)
                    euroc_cam_df.to_csv(output_cam0_csv_path, index=False, header=True)
                    print(f"  Camera CSV data converted and saved to: {output_cam0_csv_path}")
                elif image_files_to_move_and_rename: # Images were found, but CSV list ended up empty (e.g. all timestamps invalid)
                     print(f"  Camera CSV data was processed, but no valid entries to write to {output_cam0_csv_path}.")
                else:
                    print("  No camera data/images processed or found to create new CSV.")
        
        except pd.errors.EmptyDataError:
            print(f"  Error: Camera data CSV file {input_tracking_csv_path} is empty.")
        except Exception as e:
            print(f"  Error processing Camera data from {input_tracking_csv_path}: {e}")
    else:
        print(f"  Camera data CSV file not found: {input_tracking_csv_path}")

    # The original 'tracking' and 'imu_apps' directories are not deleted automatically for safety.
    # Images from 'tracking/' are moved, so it might become empty of .png files.
    # The CSVs 'tracking/data.csv' and 'imu_apps/data.csv' remain in their original locations.

    print(f"\nConversion process for {sequence_dir_path} finished.")
    print(f"Please check the '{mav0_dir}' directory for the EuRoC formatted data.")
    print("The original 'tracking/' and 'imu_apps/' directories have not been deleted.")
    print("Images have been moved from 'tracking/' to 'mav0/cam0/data/'.")

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Convert a dataset to EuRoC MAV format.\n\n"
            "The script expects the following input structure inside <sequence_path>:\n"
            "  <sequence_path>/\n"
            "    tracking/\n"
            "      data.csv\n"
            "      00000.png, 00001.png, ... (images corresponding to CSV rows)\n"
            "    imu_apps/\n"
            "      data.csv\n\n"
            "It will create the EuRoC structure within <sequence_path>:\n"
            "  <sequence_path>/\n"
            "    mav0/\n"
            "      cam0/\n"
            "        data.csv\n"
            "        data/\n"
            "          <timestamp>.png ...\n"
            "      imu0/\n"
            "        data.csv\n"
        ),
        formatter_class=argparse.RawTextHelpFormatter 
    )
    parser.add_argument(
        "sequence_path", 
        help="Full path to the sequence directory (e.g., '/path/to/data/my_sequence_01')."
    )
    args = parser.parse_args()
    
    convert_to_euroc(args.sequence_path)

if __name__ == '__main__':
    main()