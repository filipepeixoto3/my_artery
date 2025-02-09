import pandas as pd
import glob
import os
from matplotlib.path import Path

def is_inside_triangle(px, py, triangle):
    """Check if a point (px, py) is inside a triangle defined by (x0, y0), (x1, y1), (x2, y2)."""
    return Path(triangle).contains_point((px, py))

def process_detection(persons_file, sensor_files):
    # Load persons' positions
    persons_df = pd.read_csv(persons_file)
    
    detected_data = {}  # Store detections per person
    last_detection = {}  # Store last detected sensor per person
    
    # Process each car's sensor file
    for sensor_file in sensor_files:
        car_name = os.path.basename(sensor_file).split('_')[0]  # Extract car name (e.g., car0)
        sensors_df = pd.read_csv(sensor_file)
        
        for _, person in persons_df.iterrows():
            person_id = person['person_id']
            px, py, timestamp = person['x'], person['y'], person['timestamp']
            detected = False
            
            for _, sensor in sensors_df.iterrows():
                sensor_id = sensor['sensor_id']
                triangle = [(sensor['x0'], sensor['y0']),
                            (sensor['x1'], sensor['y1']),
                            (sensor['x2'], sensor['y2'])]
                
                if is_inside_triangle(px, py, triangle):
                    detected_data.setdefault(person_id, []).append([timestamp, px, py, car_name, int(sensor_id)])
                    last_detection[person_id] = (timestamp, px, py, car_name, int(sensor_id))
                    detected = True
                    break  # A person is detected by one sensor; no need to check others
            
            if not detected and person_id in last_detection:
                last_timestamp, last_px, last_py, last_car, last_sensor = last_detection[person_id]
                if timestamp - last_timestamp <= 0.5:
                    detected_data.setdefault(person_id, []).append([timestamp, px, py, last_car, int(last_sensor)])
                else:
                    del last_detection[person_id]  # Stop tracking if no detection within 0.5 sec
    
    # Save detected persons to individual files
    for person_id, data in detected_data.items():
        output_df = pd.DataFrame(data, columns=['timestamp', 'x', 'y', 'car', 'sensor'])
        output_file = f"{person_id}.csv"
        output_df.to_csv(output_file, index=False)
        print(f"Output saved to {output_file}")

if __name__ == "__main__":
    persons_file = "persons_positions.csv"
    sensor_files = glob.glob("car*_sensor_positions.csv")  # Find all sensor files
    
    process_detection(persons_file, sensor_files)
