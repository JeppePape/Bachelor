import os
import csv

def create_image_timestamp_csv(folder_path):
    """
    Creates a CSV file with timestamps and filenames from images in a folder.

    The image filenames are expected to be timestamps (e.g., "1743249165456138487.png").
    The CSV will have two columns: '#timestamp [ns]' and 'filename'.

    Args:
        folder_path (str): The path to the folder containing the images.
    """
    if not os.path.isdir(folder_path):
        print(f"Error: Folder not found at '{folder_path}'")
        return

    csv_file_name = "image_timestamps.csv"
    # Consider saving the CSV in the image folder or script's directory
    csv_file_path = os.path.join(folder_path, csv_file_name)
    # Or, to save in the script's current working directory:
    # csv_file_path = csv_file_name

    image_data = []
    found_images = False

    for item in os.listdir(folder_path):
        # Construct full file path
        item_path = os.path.join(folder_path, item)
        # Check if it's a file and ends with .png
        if os.path.isfile(item_path) and item.lower().endswith(".png"):
            found_images = True
            # Extract timestamp (filename without extension)
            timestamp_str = os.path.splitext(item)[0]
            # Validate if the timestamp is a number (optional, but good practice)
            if timestamp_str.isdigit():
                image_data.append([timestamp_str, item])
            else:
                print(f"Warning: Filename '{item}' does not seem to be a valid timestamp. Skipping.")

    if not found_images:
        print(f"No PNG images found in '{folder_path}'. CSV file will not be created.")
        return

    if not image_data:
        print(f"No PNG images with valid timestamp filenames found in '{folder_path}'. CSV file will not be created.")
        return

    try:
        with open(csv_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            # Write the header
            csv_writer.writerow(["#timestamp [ns]", "filename"])
            # Write the data rows
            csv_writer.writerows(image_data)
        print(f"Successfully created '{csv_file_path}' with {len(image_data)} entries.")
    except IOError:
        print(f"Error: Could not write to CSV file at '{csv_file_path}'. Check permissions.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    folder = input("Enter the path to the folder containing your images: ")
    create_image_timestamp_csv(folder)