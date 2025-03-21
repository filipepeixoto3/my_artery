
import pandas as pd
import numpy as np
import glob
from scipy.optimize import linear_sum_assignment

# ----------------------------
# Load tracking data from multiple cars
# ----------------------------
def load_tracks(pattern="car*_tracks_positions.csv"):
    """Load all car track position files and add 'car_id' from filename."""
    files = glob.glob(pattern)
    all_tracks = []

    for file in files:
        car_id = file.split('_')[0]  # Extract car_id from filename (e.g., car0)
        tracks = pd.read_csv(file, dtype={"track_id": int})  # Ensure track_id is integer
        tracks["car_id"] = car_id  # Add car_id column
        all_tracks.append(tracks)
    
    # Concatenate all track data into a single DataFrame
    return pd.concat(all_tracks, ignore_index=True) if all_tracks else pd.DataFrame()

# ----------------------------
# Load ground-truth pedestrian positions
# ----------------------------
def load_pedestrian_positions(pattern="personFlow*.csv"):
    """Load all pedestrian position files."""
    files = glob.glob(pattern)
    persons = {}

    for file in files:
        person_id = file.split('personFlow')[1].split('.csv')[0]   # Extract person ID (e.g., '0' from 'personFlow0.csv')
        persons[person_id] = pd.read_csv(file) # Load each person's position data into a dictionary

    return persons

# ----------------------------
# Create cost matrix based on distance and time overlap
# ----------------------------
def calculate_cost_matrix(tracks, persons, lambda_weight=100):
    """
    Create a cost matrix between tracks and ground-truth persons based on spatial distance and time penalties.
    """
    track_ids = sorted(tracks["track_id"].unique())
    person_ids = sorted(persons.keys())

    # Initialize cost matrix with infinities
    cost_matrix = np.full((len(track_ids), len(person_ids)), np.inf)
    track_sample_counts = {}  # To store the sample count for each track (used for tie-breaking)

    for i, track_id in enumerate(track_ids):
        track_data = tracks[tracks["track_id"] == track_id].copy()
        track_data["timestamp"] = track_data["timestamp"].astype(int)
        track_sample_counts[track_id] = len(track_data) # Store the sample count for each track

        for j, person_id in enumerate(person_ids):
            person_data = persons[person_id].copy()
            person_data["timestamp"] = person_data["timestamp"].astype(int)

            # Merge track and person data by timestamp
            merged = track_data.merge(person_data, on="timestamp", suffixes=("_track", "_person"))

            if merged.empty:
                continue  # Skip if there's no temporal overlap
            
            # Calculate Euclidean distances between track and person positions
            distances = np.sqrt((merged["x_track"] - merged["x_gt"])**2 +
                                (merged["y_track"] - merged["y_gt"])**2)

            mean_distance = distances.sum() / len(distances)

            # Compute time-related penalty
            track_time_min, track_time_max = track_data["timestamp"].min(), track_data["timestamp"].max()
            person_time_min, person_time_max = person_data["timestamp"].min(), person_data["timestamp"].max()
            total_time_span = max(track_time_max, person_time_max) - min(track_time_min, person_time_min)
            overlap_time = merged["timestamp"].max() - merged["timestamp"].min()
            time_penalty_1 = 1 / max(overlap_time, 0.1)
            non_overlap_time = total_time_span - overlap_time  # Time where they do NOT overlap
            time_penalty_2 = non_overlap_time  # No inversion needed, just use directly

            # Final cost combines distance and optional time penalties (currently commented out)
            distance_cost = mean_distance   # + lambda_weight * time_penalty_1 # + lambda_weight * time_penalty_2
            cost_matrix[i, j] = distance_cost

    return cost_matrix, track_ids, person_ids, track_sample_counts

# ----------------------------
# Assign tracks to persons based on cost matrix
# ----------------------------
def assign_tracks_to_persons(cost_matrix, track_ids, person_ids, tracks, persons, track_sample_counts):
    """
    Assign each track to the best matching person based on minimum cost.
    Also handles overlapping tracks and stores ghost samples (less likely matches).
    """
    assignments = {person: [] for person in person_ids}
    assigned_tracks = set()  # This will track all assigned tracks
    person_timestamps = {person: set() for person in person_ids}
    ghost_samples = []  # Stores the farther track samples

    print("Track IDs:", track_ids)
    print("Person IDs:", person_ids)

    for r, track_id in enumerate(track_ids):
        if track_id == -1:
            continue # Skip invalid track ID
        
        #print(f"\nProcessing Track {track_id} (Index {r})")

        min_cost = np.inf
        best_person = None
        best_person_sample_count = -1

        for c, person_id in enumerate(person_ids):
            cost = cost_matrix[r, c]
            #print(f"    Comparing Track {track_id} with Person {person_id}, Cost = {cost}")

            # Choose best person for this track (lowest cost or higher sample count if tie)
            if cost < min_cost and cost < np.inf:
                min_cost = cost
                best_person = person_id
                best_person_sample_count = track_sample_counts.get(track_id, 0)

            elif cost == min_cost:
                current_sample_count = track_sample_counts.get(track_id, 0)
                if current_sample_count > best_person_sample_count:
                    best_person = person_id
                    best_person_sample_count = current_sample_count

        if best_person is not None:
            # Assign track to selected person
            track_data = tracks[tracks["track_id"] == track_id]
            track_timestamps = set(track_data["timestamp"].values)

            #print(f"    Track {track_id} timestamps: {track_timestamps}")
            #print(f"    Person {best_person} timestamps: {person_timestamps[best_person]}")

            # Assign the track to the person
            assignments[best_person].append(track_id)
            assigned_tracks.add(track_id)
            person_timestamps[best_person].update(track_timestamps)

            #print(f"    Assigned Track {track_id} to Person {best_person}")

    # Handle overlapping tracks and store ghost samples
    for person_id, assigned_tracks_for_person in assignments.items():
        #print(persons)
        person_position = persons[person_id][["x_gt", "y_gt"]].values[0]

        timestamp_track_map = {}  # Map timestamps to their closest track sample

        for track_id in assigned_tracks_for_person:
            track_data = tracks[tracks["track_id"] == track_id]

            for _, row in track_data.iterrows():
                timestamp = row["timestamp"]
                track_position = np.array([row["x_track"], row["y_track"]])
                distance = np.linalg.norm(track_position - person_position)  # Compute distance to person

                # If timestamp is new, add it to the map
                if timestamp not in timestamp_track_map:
                    timestamp_track_map[timestamp] = (track_id, distance)
                else:
                    # If another track already exists at this timestamp, keep closer one and store ghost
                    existing_track, existing_distance = timestamp_track_map[timestamp]
                    if distance < existing_distance:
                        # Store ghost sample with all required values
                        ghost_samples.append({
                            "timestamp": timestamp,
                            "track_id": existing_track,
                            "x_track": tracks.loc[tracks["track_id"] == existing_track, "x_track"].values[0],
                            "y_track": tracks.loc[tracks["track_id"] == existing_track, "y_track"].values[0],
                            "car_id": tracks.loc[tracks["track_id"] == existing_track, "car_id"].values[0],
                        })
                        timestamp_track_map[timestamp] = (track_id, distance)
                    else:
                        # Store ghost sample with full details
                        ghost_samples.append({
                            "timestamp": timestamp,
                            "track_id": track_id,
                            "x_track": row["x_track"],
                            "y_track": row["y_track"],
                            "car_id": row["car_id"],
                        })
    # Print final assignment result
    print("\nFinal Assignments:")
    for person_id, assigned_tracks_for_person in assignments.items():
        print(f"    Person {person_id}: Assigned Tracks = {assigned_tracks_for_person}")

    # Save ghost_samples to a CSV file
    # Create a DataFrame with column names even if ghost_samples is empty
    column_names = ["timestamp", "track_id", "x_track", "y_track", "car_id"]
    if ghost_samples:
        df_ghosts = pd.DataFrame(ghost_samples, columns=column_names)  # Convert list to DataFrame
    else:
        # If no ghost samples, create an empty DataFrame with the same columns
        df_ghosts = pd.DataFrame(columns=column_names)
    
    # Save the DataFrame to a CSV file
    df_ghosts.to_csv("ghosts.csv", index=False)  # Save without row indices
    print("Ghost samples saved to ghosts.csv")

    return assignments

# ----------------------------
# Assign the closest track position to each pedestrian position at the same timestamp
# ----------------------------
def associate_closest_tracks(tracks, persons, assignments):
    """Associate each pedestrian position with the closest track position at the same timestamp, ensuring each track is assigned only once.
       Every person timestamp will appear in the output, even if no track is assigned."""
    
    all_timestamps = sorted(tracks["timestamp"].unique())  # Get all timestamps in order
    person_results = {person_id: [] for person_id in persons}  # Store results per person

    for timestamp in all_timestamps:
        # Get all persons and tracks at this timestamp
        person_candidates = {pid: data[data["timestamp"] == timestamp] for pid, data in persons.items()}
        track_candidates = tracks[tracks["timestamp"] == timestamp]

        associations = []

        for person_id, person_data in person_candidates.items():
            for _, person_row in person_data.iterrows():
                x_gt, y_gt = person_row["x_gt"], person_row["y_gt"]
                sensor_id = person_row.get("sensor_id", "unknown")  # Default if missing
                
                if not track_candidates.empty:
                    for _, track_row in track_candidates.iterrows():
                        track_id = track_row["track_id"]
                        x_raw, y_raw = track_row["x_raw"], track_row["y_raw"]
                        distance = np.sqrt((x_raw - x_gt) ** 2 + (y_raw - y_gt) ** 2)
                        associations.append((person_id, x_gt, y_gt, track_id, distance, x_raw, y_raw, track_row["car_id"]))

        # Sort associations by increasing distance (smallest first)
        associations.sort(key=lambda x: x[4])

        assigned_tracks = set()
        assigned_persons = set()

        for person_id, x_gt, y_gt, track_id, distance, x_raw, y_raw, car_id in associations:
            if track_id in assigned_tracks or person_id in assigned_persons:
                continue  # Skip if track or person is already assigned
            # Check if this track_id is in the list of tracks assigned to the person
            if person_id in assignments and track_id in assignments[person_id]:
                # Find the track associated with this person
                
                track_assigned = track_candidates[track_candidates["track_id"] == track_id]

                if not track_assigned.empty:
                    track_assigned = track_assigned.iloc[0]
                    x_track, y_track = track_assigned["x_track"], track_assigned["y_track"]
                else:
                    x_track, y_track = None, None
                row = {
                    "timestamp": timestamp,
                    "x_gt": x_gt,
                    "y_gt": y_gt,
                    "x_raw": x_raw,
                    "y_raw": y_raw,
                    "track_id_raw": track_id if track_id != -1 else "",  # Use "" if track_id is -1
                    "track_id": track_id if track_id != -1 else "",  # Use "" if track_id is -1
                    "x_track": x_track,
                    "y_track": y_track,
                    "car_id": car_id,
                    "sensor_id": sensor_id
                }

                person_results[person_id].append(row)
                assigned_tracks.add(track_id)
                assigned_persons.add(person_id)

    # ** Ensure all person timestamps appear, even if no track was assigned **
    for person_id, person_data in persons.items():
        for _, person_row in person_data.iterrows():
            timestamp, x_gt, y_gt, car_id = person_row["timestamp"], person_row["x_gt"], person_row["y_gt"], person_row["car_id"]
            sensor_id = person_row.get("sensor_id", "unknown")

            if not any(row["timestamp"] == timestamp for row in person_results[person_id]):
                row = {
                    "timestamp": timestamp,
                    "x_gt": x_gt,
                    "y_gt": y_gt,
                    "x_raw": None,
                    "y_raw": None,
                    "track_id_raw": "",
                    "track_id": "",
                    "x_track": None,
                    "y_track": None,
                    "car_id": car_id,
                    "sensor_id": sensor_id
                }
                person_results[person_id].append(row)

    # Save results to CSV
    for person_id, output_data in person_results.items():
        output_df = pd.DataFrame(output_data)

        # Sort by timestamp
        output_df = output_df.sort_values(by="timestamp")

        # Ensure track_id values are empty strings if they are -1
        output_df["track_id_raw"] = output_df["track_id_raw"].replace(-1, "").astype(str)
        output_df["track_id"] = output_df["track_id"].replace(-1, "").astype(str)

        filename = f"{person_id}_tracks.csv"
        output_df.to_csv(filename, index=False)
        print(f"📂 Saved {filename}")

def main():

    tracks = load_tracks("car*_tracks_positions.csv")
    if tracks.empty:
        print("❌ No track data found.")
        return

    persons = load_pedestrian_positions("personFlow*.csv")
    if not persons:
        print("❌ No pedestrian data found.")
        return

    print(f"Number of persons: {len(persons)}")
    print(f"Number of tracks: {len(tracks['track_id'].unique())}")

    # Calculate cost matrix
    cost_matrix, track_ids, person_ids, track_sample_counts = calculate_cost_matrix(tracks, persons)
    print()
    # Assign tracks to persons
    assignments = assign_tracks_to_persons(cost_matrix, track_ids, person_ids, tracks, persons, track_sample_counts)

    associate_closest_tracks(tracks, persons, assignments)

if __name__ == "__main__":
    main()
