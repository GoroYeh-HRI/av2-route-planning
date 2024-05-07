"""This script extract CityName from each log_id_*_CityName.npy and write to a json file"""

import json
import os
from pathlib import Path


def get_log_ids(directory):
    log_ids = []
    # Iterate over the contents of the directory
    for entry in os.listdir(directory):
        # Join the directory path with the entry name to get the full path
        full_path = os.path.join(directory, entry)
        # Check if the entry is a directory
        if os.path.isdir(full_path):
            log_ids.append(entry)
    return log_ids

# /home/v0392580/planning/data/av2-datasets/train/00a6ffc1-6ce9-3bc3-a060-6006e9893a1a/map/0a8a4cfa-4902-3a76-8301-08698d6290a2_ground_height_surface____PIT.npy
def extract_cityname_from_log(dataroot, log_id):
    log_map_dirpath = Path(dataroot) / log_id / "map" 
    npy_files = list(log_map_dirpath.glob("*.npy"))
    if len(npy_files) >1:
        print(f"ERROR! log_id {log_id} contains multiple npy file. Can't extract CityName")
    for npy_file in npy_files:
        cityname = str(npy_file)[-7:-4]
    return cityname

def write_dict_to_json(hash_table, json_file):
    # Write the dictionary to a JSON file
    with open(json_file, "w") as json_file_object:
        json_file_object.write("{\n")
        for key, value in hash_table.items():
            json_file_object.write(f'"{key}": "{value}",\n')        
        # json.dump(hash_table, json_file_object)
        json_file_object.write("}\n")

# Miami: a list of log_ids -> txt
def write_log_ids_to_txt(log_ids, txt_file):
    with open(txt_file, "w") as f:
        for log_id in log_ids:
            f.write(f"{log_id}\n")
    f.close()

def main():
    dataroot = "/data1/av2-datasets/train"  # Replace with the path to your directory
    json_file = "log-cityname.json"

    log_to_cityname = {}
    mia_log_ids = []

    log_ids = get_log_ids(dataroot)
    for log_id in log_ids:
        cityname = extract_cityname_from_log(dataroot, log_id)
        print(f"    Map {log_id} -> CityName {cityname}")
        log_to_cityname[log_id] = cityname
        if cityname == "MIA":
            mia_log_ids.append(log_id)
    
    write_dict_to_json(log_to_cityname, json_file)
    write_log_ids_to_txt(mia_log_ids, "log-ids-mia.txt")




# Check if the script is being run directly
if __name__ == "__main__":
    # If the script is being run directly, execute the main function
    main()