import numpy as np
import matplotlib.pyplot as plt
import mplcursors
import pandas as pd
import glob

# Kalman filter implementation
def kalman_filter(detections):
    """Apply Kalman Filter to the given detections."""
    x = detections[0]  # Initial position estimate
    P = np.eye(2)  # Initial covariance matrix
    F = np.eye(2)  # State transition matrix
    Q = np.eye(2) * 0.1  # Process noise covariance matrix
    H = np.eye(2)  # Measurement matrix
    R = np.eye(2) * 0.1  # Measurement noise covariance matrix
    
    filtered_positions = []
    
    for z in detections:
        # Predict
        x = F @ x
        P = F @ P @ F.T + Q
        
        # Update
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ y
        P = (np.eye(2) - K @ H) @ P
        
        filtered_positions.append(x.copy())
    
    return np.array(filtered_positions)

# Load data from CSV
def load_data(file_path):
    """Load data from a CSV file into a DataFrame."""
    return pd.read_csv(file_path)

# Plot the original detections and Kalman-filtered positions
def plot_detections_and_kalman(ax, detections, kalman_positions, timestamps, positions_df, file_data, color, label_prefix):
    """Plot the detections and Kalman-filtered points with connecting lines to positions."""
    # Plot original detections
    scatter = ax.scatter(detections[:, 0], detections[:, 1], color=color, s=5, label=f'{label_prefix} Detections')
    scatter.set_gid(f'{label_prefix}_detections')

    # Plot Kalman-filtered positions
    kalman_scatter = ax.scatter(kalman_positions[:, 0], kalman_positions[:, 1], color=color, marker='x', s=30, label=f'{label_prefix} Kalman')
    kalman_scatter.set_gid(f'{label_prefix}_kalman')

    # Match detections with positions based on timestamp
    for idx, pos_row in positions_df.iterrows():
        pos_timestamp = pos_row['timestamp']
        detection_match = file_data[file_data['timestamp'] == pos_timestamp]
        if not detection_match.empty:
            detection_pos = detection_match[['x', 'y']].values[0]
            ax.plot([detection_pos[0], pos_row['x']], [detection_pos[1], pos_row['y']], color=color, lw=0.5)

    # Match detections with Kalman predictions based on timestamp
    for idx, row in file_data.iterrows():
        timestamp = row['timestamp']
        kalman_match = kalman_positions[idx]  # Corresponding Kalman prediction
        detection_pos = row[['x', 'y']].values
        ax.plot([detection_pos[0], kalman_match[0]], [detection_pos[1], kalman_match[1]], color=color, linestyle='--', lw=0.5)

    # Store data for hover annotations
    data[f'{label_prefix}_detections'] = pd.DataFrame({'x': detections[:, 0], 'y': detections[:, 1], 'timestamp': timestamps})
    data[f'{label_prefix}_kalman'] = pd.DataFrame({'x': kalman_positions[:, 0], 'y': kalman_positions[:, 1], 'timestamp': timestamps})

# Plot route from positions.txt
def plot_route(ax, route_data, color):
    """Plot a route on the given axis."""
    groups = route_data.groupby('id')
    for name, group in groups:
        line, = ax.plot(group['x'], group['y'], color=color, lw=0.5, marker='o', markersize=0.7, label=f"Route {name}")
        line.set_gid(name)  # Set gid to route id
        
        # Store the route information properly in the global data dictionary
        route_df = pd.DataFrame({
            'x': group['x'].values,
            'y': group['y'].values,
            'timestamp': group['timestamp'].values
        })
        
        # Ensure correct assignment
        data[name] = route_df  # Store route data under route ID
        
        # Store the line object in line_objects for interactive updates
        line_objects[name] = line

# Handle hover annotation
def on_add(sel):
    """Handle cursor hover events to display annotation and adjust markers."""
    artist = sel.artist
    index = int(round(sel.index))  # Convert the float index to an integer
    gid = artist.get_gid()

    if gid in data:
        row = data[gid].iloc[index]
        timestamp = row['timestamp']
        sel.annotation.set(
            text=f"X: {row['x']}\n"
                 f"Y: {row['y']}\n"
                 f"Timestamp: {timestamp}"
        )

# Main function to read data, apply Kalman filter, and plot results
def plot_data():
    """Plot all data files and handle interactions."""
    global data, line_objects
    
    fig, ax = plt.subplots()
    
    # Initialize the global data structure
    data = {}  # Ensure it's reset each time this function is called
    line_objects = {}  # Also reset the line_objects dictionary
    
    # Plot positions.txt data with multiple routes
    positions_data = load_data('positions.txt')
    unique_routes = positions_data['id'].unique()
    colors = plt.cm.jet(np.linspace(0, 1, len(unique_routes)))  # Generate a color map for the routes

    for i, route_id in enumerate(unique_routes):
        route_data = positions_data[positions_data['id'] == route_id]
        
        # Store route data (x, y, timestamp) for hover annotations
        plot_route(ax, route_data, colors[i])
    
    # Plot detection files with Kalman filter applied
    detection_files = glob.glob('car*_detection_pos.txt')
    detection_colors = plt.cm.tab10(np.linspace(0, 1, len(detection_files)))  # Different color map for detection files
    
    for i, file in enumerate(detection_files):
        file_data = load_data(file)
        timestamps = file_data['timestamp']
        kalman_predictions = kalman_filter(file_data[['x', 'y']].values)  # Apply Kalman filter

        # Store detection data and Kalman predictions (including timestamps)
        detection_df = pd.DataFrame({
            'x': file_data['x'].values, 
            'y': file_data['y'].values, 
            'timestamp': file_data['timestamp'].values,
            'kalman_x': kalman_predictions[:, 0],
            'kalman_y': kalman_predictions[:, 1]
        })
        data[file] = detection_df
        
        # Plot original detections and Kalman-filtered positions
        plot_detections_and_kalman(ax, file_data[['x', 'y']].values, kalman_predictions, timestamps, positions_data, file_data, detection_colors[i], file)
        
    # Add interactive annotations
    mplcursors.cursor(hover=True).connect("add", on_add)
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Vehicle and Pedestrian Detections with Kalman Filter')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    data = {}
    plot_data()
