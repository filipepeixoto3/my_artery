###### EACH PERSON HAS MANY TRACKS, THE ASSOCIATION IS BASED ON THE POSITION
# import pandas as pd
# import glob
# import os
# from scipy.spatial import KDTree

# def associate_tracks_to_persons(person_files, track_files, time_threshold=0.5, distance_threshold=1.0):
#     for person_file in person_files:
#         person_id = os.path.splitext(os.path.basename(person_file))[0]
#         person_df = pd.read_csv(person_file)
        
#         associated_tracks = []
        
#         for track_file in track_files:
#             car_id = os.path.basename(track_file).split('_')[0]
#             track_df = pd.read_csv(track_file)
            
#             if person_df.empty or track_df.empty:
#                 continue
            
#             # Build a KDTree for fast spatial search
#             person_tree = KDTree(person_df[['x', 'y']].values)
            
#             for _, track in track_df.iterrows():
#                 track_x, track_y, track_time = track['x'], track['y'], track['timestamp']
#                 sensor_id, track_id = int(track['sensor_id']), track['track_id']
                
#                 # Find the closest person position in time
#                 time_diff = abs(person_df['timestamp'] - track_time)
#                 closest_idx = time_diff.idxmin()
#                 closest_time = time_diff.min()
                
#                 if closest_time <= time_threshold:
#                     groundtruth_x, groundtruth_y = person_df.loc[closest_idx, ['x', 'y']]
#                     distance = ((groundtruth_x - track_x)**2 + (groundtruth_y - track_y)**2) ** 0.5
                    
#                     if distance <= distance_threshold:
#                         associated_tracks.append([
#                             track_id, car_id, sensor_id, track_time, groundtruth_x, groundtruth_y, track_x, track_y
#                         ])
        
#         # Save output file for this person
#         output_file = f"{person_id}_tracks.csv"
#         output_df = pd.DataFrame(associated_tracks, columns=[
#             'track_id', 'car_id', 'sensor_id', 'timestamp', 'groundtruth_x', 'groundtruth_y', 'track_x', 'track_y'
#         ])
#         output_df.to_csv(output_file, index=False)
#         print(f"Output saved to {output_file}")

# if __name__ == "__main__":
#     person_files = glob.glob("personFlow*.csv")  # Assuming all [person_id].csv files are in the directory
#     track_files = glob.glob("car*_tracks_positions.csv")  # Find all track position files
    
#     associate_tracks_to_persons(person_files, track_files)






###### EACH PERSON HAS ONLY ONE TRACK
# import os
# import pandas as pd
# import glob
# from scipy.spatial.distance import euclidean

# def find_best_track(person_df, track_dfs):
#     """Find the best track match for a person based on timestamp and proximity."""
#     best_match = None
#     best_distance = float('inf')
    
#     for car_id, track_df in track_dfs.items():
#         for _, track in track_df.groupby('track_id'):
#             common_timestamps = set(person_df['timestamp']).intersection(set(track['timestamp']))
#             if not common_timestamps:
#                 continue
            
#             distances = [
#                 euclidean(person_df.loc[person_df['timestamp'] == ts, ['x', 'y']].values[0],
#                           track.loc[track['timestamp'] == ts, ['x', 'y']].values[0])
#                 for ts in common_timestamps
#             ]
#             avg_distance = sum(distances) / len(distances)
            
#             if avg_distance < best_distance:
#                 best_distance = avg_distance
#                 best_match = (car_id, track['track_id'].iloc[0], track)
    
#     return best_match

# def process_files():
#     person_files = glob.glob("personFlow*.csv")
#     car_files = glob.glob("*_tracks_positions.csv")
    
#     # Load person data
#     persons = {}
#     for file in person_files:
#         if file.endswith('_tracks.csv') or '-' in file:
#             continue  # Skip output and car track files
#         person_id = file.replace('.csv', '').split('personFlow')[-1]
#         persons[person_id] = pd.read_csv(file)
    
#     # Load car track data
#     tracks = {}
#     for file in car_files:
#         car_id = file.split('-')[0]
#         tracks[car_id] = pd.read_csv(file)
    
#     print(f"Number of persons: {len(persons)}")
#     print(f"Number of tracks: {sum(len(df.groupby('track_id')) for df in tracks.values())}")
    
#     assigned_tracks = {}
#     unassigned_tracks = []
    
#     # Assign one track per person
#     for person_id, person_df in persons.items():
#         best_match = find_best_track(person_df, tracks)
#         if best_match:
#             car_id, track_id, track_df = best_match
#             assigned_tracks[person_id] = track_df
#             tracks[car_id] = tracks[car_id][tracks[car_id]['track_id'] != track_id]
#         else:
#             assigned_tracks[person_id] = None
    
#     # Assign leftover tracks to extend existing ones
#     for car_id, track_df in tracks.items():
#         for _, track in track_df.groupby('track_id'):
#             unassigned_tracks.append(track)
    
#     for person_id, assigned_track in assigned_tracks.items():
#         if assigned_track is None:
#             continue
        
#         for track in unassigned_tracks:
#             if assigned_track['track_id'].iloc[0] == track['track_id'].iloc[0]:
#                 assigned_tracks[person_id] = pd.concat([assigned_track, track]).sort_values('timestamp')
    
#     # Save results
#     for person_id, track_df in assigned_tracks.items():
#         if track_df is None:
#             continue
#         person_df = persons[person_id]
#         output_data = []
        
#         for _, row in person_df.iterrows():
#             matching_track = track_df[track_df['timestamp'] == row['timestamp']]
#             if not matching_track.empty:
#                 output_data.append([
#                     matching_track['track_id'].values[0],
#                     matching_track['sensor_id'].values[0],
#                     matching_track['timestamp'].values[0],
#                     row['x'], row['y'],
#                     matching_track['x'].values[0], matching_track['y'].values[0]
#                 ])
        
#         output_df = pd.DataFrame(output_data, columns=['track_id', 'sensor_id', 'timestamp', 'gt_x', 'gt_y', 'track_x', 'track_y'])
#         output_df.to_csv(f"{person_id}_tracks.csv", index=False)

# if __name__ == "__main__":
#     process_files()








import os
import pandas as pd
import glob
from scipy.spatial.distance import euclidean

def find_best_track(person_df, track_dfs):
    """Find the best track match for a person based on timestamp and proximity.""" 
    best_match = None
    best_distance = float('inf')
    
    for car_id, track_df in track_dfs.items():
        for _, track in track_df.groupby('track_id'):
            common_timestamps = set(person_df['timestamp']).intersection(set(track['timestamp']))
            if not common_timestamps:
                continue
            
            distances = [
                euclidean(person_df.loc[person_df['timestamp'] == ts, ['x', 'y']].values[0],
                          track.loc[track['timestamp'] == ts, ['x', 'y']].values[0])
                for ts in common_timestamps
            ]
            avg_distance = sum(distances) / len(distances)
            
            if avg_distance < best_distance:
                best_distance = avg_distance
                best_match = (car_id, track['track_id'].iloc[0], track)
    
    return best_match

def process_files():
    # Load person and track files
    person_files = glob.glob("personFlow*.csv")
    car_files = glob.glob("*_tracks_positions.csv")
    
    # Load person data
    persons = {}
    for file in person_files:
        if file.endswith('_tracks.csv') or '-' in file:
            continue  # Skip output and car track files
        person_id = file.replace('.csv', '').split('personFlow')[-1]
        persons[person_id] = pd.read_csv(file)
    
    # Load car track data
    tracks = {}
    for file in car_files:
        car_id = os.path.splitext(os.path.basename(file))[0].split('_')[0]
        tracks[car_id] = pd.read_csv(file)
    
    print(f"Number of persons: {len(persons)}")
    print(f"Number of tracks: {sum(len(df.groupby('track_id')) for df in tracks.values())}")
    
    assigned_tracks = {person_id: [] for person_id in persons}  # Initialize a list for tracks per person
    unassigned_tracks = []  # Will hold tracks that are not yet assigned
    
    # First assign one track to each person
    for person_id, person_df in persons.items():
        best_match = find_best_track(person_df, tracks)
        if best_match:
            car_id, track_id, track_df = best_match
            assigned_tracks[person_id].append((car_id, track_id, track_df))
            tracks[car_id] = tracks[car_id][tracks[car_id]['track_id'] != track_id]  # Remove assigned track
        else:
            unassigned_tracks.append(person_id)  # If no track found, add person to unassigned list

    # Now, assign the remaining tracks to persons who don't have two tracks yet
    for person_id, person_df in persons.items():
        if len(assigned_tracks[person_id]) < 2:  # If person has only one track assigned
            # Try to assign remaining tracks
            for _ in range(2 - len(assigned_tracks[person_id])):  # Add up to 2 tracks per person
                best_match = find_best_track(person_df, tracks)
                if best_match:
                    car_id, track_id, track_df = best_match
                    assigned_tracks[person_id].append((car_id, track_id, track_df))
                    tracks[car_id] = tracks[car_id][tracks[car_id]['track_id'] != track_id]  # Remove assigned track
                else:
                    break  # No more tracks to assign
    
    # Print out the assigned tracks per person for debugging
    for person_id, tracks_data in assigned_tracks.items():
        print(f"Person ID: {person_id}, Assigned Track IDs: {[track[1] for track in tracks_data]}")
    
    # Save results
    for person_id, tracks_data in assigned_tracks.items():
        if not tracks_data:
            continue  # Skip if no tracks assigned
        
        person_df = persons[person_id]
        output_data = []
        
        for _, row in person_df.iterrows():
            for car_id, track_id, track_df in tracks_data:
                matching_track = track_df[track_df['timestamp'] == row['timestamp']]
                if not matching_track.empty:
                    output_data.append([track_id, car_id, matching_track['sensor_id'].values[0],
                                        row['timestamp'], row['x'], row['y'],
                                        matching_track['x'].values[0], matching_track['y'].values[0]])
        
        output_df = pd.DataFrame(output_data, columns=['track_id', 'car_id', 'sensor_id', 'timestamp', 'gt_x', 'gt_y', 'track_x', 'track_y'])
        output_df.to_csv(f"{person_id}_tracks.csv", index=False)
        print(f"Output saved for Person ID: {person_id}")

if __name__ == "__main__":
    process_files()
