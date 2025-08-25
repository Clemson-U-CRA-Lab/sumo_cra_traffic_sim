# /usr/bin/env python3

from EcoNN_utils import *
import os
import glob

def main(dir_path, output_dir):
    try:
        EcoNN_controller = NN_controller(nn_pt_file='traffic_following_control_dc_trained.pt')
        lastest_input_processed = False
        while True:
            traffic_info_files = glob.glob(os.path.join(dir_path, "*"))
            if not traffic_info_files:
                continue
            else:
                latest_file = max(traffic_info_files, key=os.path.getctime)
                vehicle_id, s_att, frame_id = nn_controller_step(EcoNN_controller, latest_file)
                control_filename = os.path.join(output_dir, f"eco_nn_control_{frame_id}.csv")
                
                if os.path.exists(control_filename):
                    if lastest_input_processed:
                        continue
                    else:
                        print(f"{control_filename} already exists. Skipping write.")
                        lastest_input_processed = True
                        continue
                else:
                    with open(control_filename, mode='w', newline='') as control_file:
                        writer = csv.writer(control_file)
                        writer.writerow(['Vehicle_ID', 'EcoNN_Acceleration'])
                        for vid, acc in zip(vehicle_id, s_att):
                            writer.writerow([vid, acc])
                    print(f"Control file {control_filename} written successfully.")
                    
    except KeyboardInterrupt:
        print("Exiting...")
        pass
    
if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'traffic_info')
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'control_output')
    main(dir_path, output_dir)