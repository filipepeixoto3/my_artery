# import pandas as pd
# import numpy as np
# import glob
# from scipy.optimize import linear_sum_assignment

# def load_tracks(file):
#     """ Load track data from CSV file. """
#     try:
#         tracks = pd.read_csv(file, dtype={"track_id": str})
#     except Exception as e:
#         print(f"Error loading {file}: {e}")
#         return None
#     return tracks

# def load_pedestrian_positions(pattern):
#     """ Load all pedestrian position files matching a pattern. """
#     files = glob.glob(pattern)
#     persons = {}
#     for file in files:
#         person_id = file.split('personFlow')[1].split('.csv')[0]  # Keep as string
#         persons[person_id] = pd.read_csv(file, dtype={"track_id": str})
#     return persons

# def calculate_cost_matrix(tracks, persons, lambda_weight=100):
#     track_ids = sorted(tracks["track_id"].unique())
#     person_ids = sorted(persons.keys())

#     cost_matrix = np.full((len(track_ids), len(person_ids)), np.inf)

#     print("\n🚨 Cost Matrix Calculation 🚨")
#     for i, track_id in enumerate(track_ids):
#         track_data = tracks[tracks["track_id"] == track_id]

#         for j, person_id in enumerate(person_ids):
#             person_data = persons[person_id]

#             track_time_range = (track_data["timestamp"].min(), track_data["timestamp"].max())
#             person_time_range = (person_data["timestamp"].min(), person_data["timestamp"].max())

#             # Merge timestamps to ensure alignment
#             merged = track_data.merge(person_data, on="timestamp", suffixes=("_track", "_person"))

#             if merged.empty:
#                 continue

#             # Compute Euclidean distance (sum of distances for each match)
#             distances = np.sqrt((merged["x_track"] - merged["x_person"])**2 +
#                                 (merged["y_track"] - merged["y_person"])**2)

#             # Compute the mean distance (sum of distances divided by number of matches)
#             mean_distance = distances.sum() / len(distances)

#             # Add penalty for short overlaps based on the time range of the overlap
#             overlap_time = merged["timestamp"].max() - merged["timestamp"].min()
#             time_penalty = 1 / max(overlap_time, 0.1)  # Avoid division by zero

#             # Cost Calculation: Mean Distance + Time Penalty
#             distance_cost = mean_distance + lambda_weight * time_penalty

#             cost_matrix[i, j] = distance_cost

#             # Debugging: print out the cost for each track-person pair
#             print(f"Track {track_id} -> Person {person_id}: Distance = {mean_distance}, Time Penalty = {time_penalty}, Total Cost = {distance_cost}")

#     print("\n🚨 Cost Matrix Finished 🚨")
#     print(cost_matrix)

#     return cost_matrix, track_ids, person_ids

# def assign_tracks_to_persons(cost_matrix, track_ids, person_ids, tracks, persons):
#     """ Assigns tracks to persons ensuring no timestamp overlaps for multiple assignments. """
#     row_ind, col_ind = linear_sum_assignment(cost_matrix)

#     assignments = {}
#     assigned_tracks = set()
#     invisible_persons_data = []
#     person_timestamps = {person: set() for person in person_ids}  # Stores timestamps already assigned to a person

#     # Initial assignment using the Hungarian algorithm
#     for r, c in zip(row_ind, col_ind):
#         if cost_matrix[r, c] < np.inf:
#             track_id = track_ids[r]
#             person_id = person_ids[c]
#             track_data = tracks[tracks["track_id"] == track_id]

#             # Check for timestamp overlap
#             track_timestamps = set(track_data["timestamp"].values)
#             if track_timestamps & person_timestamps[person_id]:  # Overlapping timestamps exist
#                 continue  # Skip this assignment

#             # Assign the track and update the timestamp record for this person
#             assignments[track_id] = person_id
#             assigned_tracks.add(track_id)
#             person_timestamps[person_id].update(track_timestamps)

#     # Find tracks that were NOT assigned
#     unassigned_tracks = set(track_ids) - assigned_tracks

#     # Loop through unassigned tracks and attempt reassignment
#     for track in unassigned_tracks:
#         track_index = track_ids.index(track)
#         best_person_index = np.argmin(cost_matrix[track_index])  # Best person (lowest cost)
#         best_person = person_ids[best_person_index]

#         # Ensure the new track does not overlap timestamps with the already assigned tracks
#         track_data = tracks[tracks["track_id"] == track]
#         track_timestamps = set(track_data["timestamp"].values)

#         # Try to find the first person that does not have timestamp overlap
#         reassigned = False
#         for person_index in np.argsort(cost_matrix[track_index]):  # Try second-best, third-best, etc.
#             person_id = person_ids[person_index]

#             # Check for timestamp overlap with this person
#             if not (track_timestamps & person_timestamps[person_id]):  # No timestamp overlap
#                 assignments[track] = person_id
#                 print(f"✅ Track {track} was reassigned to Person {person_id}.")
#                 person_timestamps[person_id].update(track_timestamps)
#                 reassigned = True
#                 break  # Exit loop once a valid person is found

#         if not reassigned:
#             print(f"❌ Track {track} remains unassigned due to timestamp overlap with all possible persons.")

#     return assignments



# def save_unassigned_tracks(tracks, unassigned_tracks, filename="ghosts.csv"):
#     """ Save unassigned tracks to a CSV file. """
#     ghost_tracks = tracks[tracks["track_id"].isin(unassigned_tracks)]
    
#     if ghost_tracks.empty:
#         print("\n🎉 No ghost persons! All tracks were assigned.")
#     else:
#         ghost_tracks.to_csv(filename, index=False)
#         print(f"\n👻 Ghost persons saved to {filename}. {len(ghost_tracks['track_id'].unique())} tracks are unassigned.")


# def main():
#     # Load the tracks
#     tracks = load_tracks("car0_tracks_positions.csv")
    
#     if isinstance(tracks, pd.DataFrame):
#         print(f"Tracks loaded successfully as DataFrame. Number of tracks: {len(tracks)}")
#     else:
#         print("Error: Tracks are not loaded as DataFrame.")
#         return

#     persons = load_pedestrian_positions("personFlow*.csv")

#     print(f"Number of persons: {len(persons)}")
#     print(f"Number of tracks: {len(tracks['track_id'].unique())}")

#     # Calculate cost matrix
#     cost_matrix, track_ids, person_ids = calculate_cost_matrix(tracks, persons)

#     # Assign tracks to persons with timestamp validation
#     assignments = assign_tracks_to_persons(cost_matrix, track_ids, person_ids, tracks, persons)

#     # Find unassigned tracks
#     assigned_tracks = set(assignments.keys())
#     unassigned_tracks = set(track_ids) - assigned_tracks

#     # Save ghost persons to CSV
#     save_unassigned_tracks(tracks, unassigned_tracks)

#     # Output the number of assigned tracks
#     print(f"\n✅ Number of assigned tracks: {len(assignments)}")
    
#     # Output the assignments
#     print("\n📌 Final Track to Person Assignments:")
#     for track, person in assignments.items():
#         print(f"Track {track} -> Person {person}")

# if __name__ == "__main__":
#     main()

import pandas as pd
import numpy as np
import glob
from scipy.optimize import linear_sum_assignment

def load_tracks(file):
    """ Load track data from CSV file. """
    try:
        tracks = pd.read_csv(file, dtype={"track_id": str})
    except Exception as e:
        print(f"Error loading {file}: {e}")
        return None
    return tracks

def load_pedestrian_positions(pattern):
    """ Load all pedestrian position files matching a pattern. """
    files = glob.glob(pattern)
    persons = {}
    for file in files:
        person_id = file.split('personFlow')[1].split('.csv')[0]  # Extract numeric part only
        persons[person_id] = pd.read_csv(file, dtype={"track_id": str})
    return persons

def calculate_cost_matrix(tracks, persons, lambda_weight=100):
    track_ids = sorted(tracks["track_id"].unique())
    person_ids = sorted(persons.keys())

    cost_matrix = np.full((len(track_ids), len(person_ids)), np.inf)

    for i, track_id in enumerate(track_ids):
        track_data = tracks[tracks["track_id"] == track_id].copy()
        track_data["timestamp"] = track_data["timestamp"].astype(int)

        for j, person_id in enumerate(person_ids):
            person_data = persons[person_id].copy()
            person_data["timestamp"] = person_data["timestamp"].astype(int)

            merged = track_data.merge(person_data, on="timestamp", suffixes=("_track", "_person"))

            if merged.empty:
                continue

            distances = np.sqrt((merged["x_track"] - merged["x_person"])**2 +
                                (merged["y_track"] - merged["y_person"])**2)

            mean_distance = distances.sum() / len(distances)

            overlap_time = merged["timestamp"].max() - merged["timestamp"].min()
            time_penalty = 1 / max(overlap_time, 0.1)

            distance_cost = mean_distance + lambda_weight * time_penalty
            cost_matrix[i, j] = distance_cost

    return cost_matrix, track_ids, person_ids

def assign_tracks_to_persons(cost_matrix, track_ids, person_ids, tracks, persons):
    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    assignments = {}
    assigned_tracks = set()
    person_timestamps = {person: set() for person in person_ids}

    for r, c in zip(row_ind, col_ind):
        if cost_matrix[r, c] < np.inf:
            track_id = track_ids[r]
            person_id = person_ids[c]
            track_data = tracks[tracks["track_id"] == track_id]

            track_timestamps = set(track_data["timestamp"].values)
            if track_timestamps & person_timestamps[person_id]:
                continue  

            assignments[track_id] = person_id
            assigned_tracks.add(track_id)
            person_timestamps[person_id].update(track_timestamps)

    return assignments

def save_unassigned_tracks(tracks, unassigned_tracks, filename="ghosts.csv"):
    """ Save unassigned tracks to a CSV file. """
    ghost_tracks = tracks[tracks["track_id"].isin(unassigned_tracks)]
    
    if ghost_tracks.empty:
        print("\n🎉 No ghost persons! All tracks were assigned.")
    else:
        ghost_tracks.to_csv(filename, index=False)
        print(f"\n👻 Ghost persons saved to {filename}. {len(ghost_tracks['track_id'].unique())} tracks are unassigned.")

def save_tracks_by_person(assignments, tracks, persons):
    """ Save assigned tracks to separate CSVs for each person. """
    person_data = {person_id: persons[person_id] for person_id in persons}

    for track_id, person_id in assignments.items():
        track_data = tracks[tracks["track_id"] == track_id].copy()
        person_data_df = person_data[person_id].copy()

        # Ensure timestamps are integers
        track_data["timestamp"] = track_data["timestamp"].astype(int)
        person_data_df["timestamp"] = person_data_df["timestamp"].astype(int)

        # Merge track and person data
        merged = track_data.merge(person_data_df, on="timestamp", suffixes=("_track", "_gt"))

        if merged.empty:
            continue  

        # Extract relevant columns
        merged["car_id"] = track_id
        merged["sensor_id"] = person_data_df["sensor_id"].iloc[0] if "sensor_id" in person_data_df else "unknown"

        output_columns = ["timestamp", "car_id", "sensor_id", "x_gt", "y_gt", "x_track", "y_track"]
        merged = merged.rename(columns={"x_person": "x_gt", "y_person": "y_gt", "x_track": "x_track", "y_track": "y_track"})

        filename = f"{person_id}_tracks.csv"
        merged[output_columns].to_csv(filename, index=False)
        print(f"📂 Saved {filename}")

def main():
    # Load the tracks
    tracks = load_tracks("car0_tracks_positions.csv")
    
    if isinstance(tracks, pd.DataFrame):
        print(f"Tracks loaded successfully. Number of tracks: {len(tracks)}")
    else:
        print("Error: Tracks are not loaded.")
        return

    persons = load_pedestrian_positions("personFlow*.csv")

    print(f"Number of persons: {len(persons)}")
    print(f"Number of tracks: {len(tracks['track_id'].unique())}")

    # Calculate cost matrix
    cost_matrix, track_ids, person_ids = calculate_cost_matrix(tracks, persons)

    # Assign tracks to persons
    assignments = assign_tracks_to_persons(cost_matrix, track_ids, person_ids, tracks, persons)

    # Find unassigned tracks
    assigned_tracks = set(assignments.keys())
    unassigned_tracks = set(track_ids) - assigned_tracks

    # Save ghost persons
    save_unassigned_tracks(tracks, unassigned_tracks)

    # Save assigned tracks per person
    save_tracks_by_person(assignments, tracks, persons)

    # Output the number of assigned tracks
    print(f"\n✅ Number of assigned tracks: {len(assignments)}")

    # Output the assignments
    print("\n📌 Final Track to Person Assignments:")
    for track, person in assignments.items():
        print(f"Track {track} -> Person {person}")

if __name__ == "__main__":
    main()
