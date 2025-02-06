import pandas as pd
import glob
import os
from scipy.spatial import KDTree

def associate_tracks_to_persons(person_files, track_files, time_threshold=0.5, distance_threshold=1.0):
    for person_file in person_files:
        person_id = os.path.splitext(os.path.basename(person_file))[0]
        person_df = pd.read_csv(person_file)
        
        associated_tracks = []
        
        for track_file in track_files:
            car_id = os.path.basename(track_file).split('_')[0]
            track_df = pd.read_csv(track_file)
            
            if person_df.empty or track_df.empty:
                continue
            
            # Build a KDTree for fast spatial search
            person_tree = KDTree(person_df[['x', 'y']].values)
            
            for _, track in track_df.iterrows():
                track_x, track_y, track_time = track['x'], track['y'], track['timestamp']
                sensor_id, track_id = track['sensor_id'], track['track_id']
                
                # Find the closest person position in time
                time_diff = abs(person_df['timestamp'] - track_time)
                closest_idx = time_diff.idxmin()
                closest_time = time_diff.min()
                
                if closest_time <= time_threshold:
                    groundtruth_x, groundtruth_y = person_df.loc[closest_idx, ['x', 'y']]
                    distance = ((groundtruth_x - track_x)**2 + (groundtruth_y - track_y)**2) ** 0.5
                    
                    if distance <= distance_threshold:
                        associated_tracks.append([
                            track_id, car_id, sensor_id, track_time, groundtruth_x, groundtruth_y, track_x, track_y
                        ])
        
        # Save output file for this person
        output_file = f"{person_id}_tracks.txt"
        output_df = pd.DataFrame(associated_tracks, columns=[
            'track_id', 'car_id', 'sensor_id', 'timestamp', 'groundtruth_x', 'groundtruth_y', 'track_x', 'track_y'
        ])
        output_df.to_csv(output_file, index=False)
        print(f"Output saved to {output_file}")

if __name__ == "__main__":
    person_files = glob.glob("*.txt")  # Assuming all [person_id].txt files are in the directory
    track_files = glob.glob("car*_tracks_positions.txt")  # Find all track position files
    
    associate_tracks_to_persons(person_files, track_files)
