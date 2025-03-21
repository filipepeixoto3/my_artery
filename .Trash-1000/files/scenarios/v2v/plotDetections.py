import matplotlib.pyplot as plt
import mplcursors
import pandas as pd
import numpy as np
import glob

data = {}  # Global variable to store data
line_objects = {}  # Global variable to store line objects for routes

def load_data(file_path):
    """Load data from a CSV file into a DataFrame."""
    return pd.read_csv(file_path)

def plot_route(ax, data, color):
    """Plot a route on the given axis."""
    groups = data.groupby('id')
    for name, group in groups:
        line, = ax.plot(group['x'], group['y'], color=color, lw=0.5, marker='o', markersize=0.7, label=f"Route {name}")
        line.set_gid(name)  # Set gid to route id
        line_objects[name] = line  # Store the line object

def plot_file(ax, file_path, color):
    """Plot data from a file with a specific color."""
    file_data = load_data(file_path)
    scatter = ax.scatter(file_data['x'], file_data['y'], color=color, s=5, label=file_path)
    scatter.set_gid(file_path)  # Set gid to file path

def on_add(sel):
    """Handle cursor hover events to display annotation and adjust markers."""
    artist = sel.artist
    index = int(round(sel.index))  # Convert the float index to an integer

    gid = artist.get_gid()
    
    if gid in data:
        # Handle detection files
        file_data = data[gid]
        if index >= len(file_data):
            return
        row = file_data.iloc[index]
        timestamp = row['timestamp']
        sel.annotation.set(
            text=f"ID: {row['id']}\n"
                 f"X: {row['x']}\n"
                 f"Y: {row['y']}\n"
                 f"Timestamp: {timestamp}"
        )
    else:
        # Handle route lines
        route_id = gid
        if route_id in data:
            route_data = data[route_id]
            if index >= len(route_data):
                return
            row = route_data.iloc[index]
            timestamp = row['timestamp']
            sel.annotation.set(
                text=f"Route: {route_id}\n"
                     f"Timestamp: {timestamp}"
            )
            
            # Update markers' size for the selected route
            for rid, line in line_objects.items():
                if rid == route_id:
                    line.set_markersize(6)  # Increase marker size for the selected route
                else:
                    line.set_markersize(2)  # Default marker size for unselected routes
            
            plt.draw()  # Redraw the plot to update marker sizes

def plot_data():
    """Plot all data files and handle interactions."""
    global data, line_objects
    
    fig, ax = plt.subplots()
    
    # Plot positions.txt data with multiple routes
    positions_data = load_data('positions.txt')
    unique_routes = positions_data['id'].unique()
    colors = plt.cm.jet(np.linspace(0, 1, len(unique_routes)))  # Generate a color map
    
    for i, route_id in enumerate(unique_routes):
        route_data = positions_data[positions_data['id'] == route_id]
        data[route_id] = route_data  # Store route data in global dictionary
        plot_route(ax, route_data, colors[i])
    
    # Plot other detection files with different colors
    detection_files = glob.glob('car1_detection_pos3.txt')
    detection_colors = plt.cm.tab10(np.linspace(0, 1, len(detection_files)))  # Different color map for other files
    
    for i, file in enumerate(detection_files):
        file_data = load_data(file)
        data[file] = file_data  # Store file data in global dictionary
        plot_file(ax, file, detection_colors[i])
    
    # Add interactive annotations
    mplcursors.cursor(hover=True).connect("add", on_add)
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Vehicle and Pedestrian Detections')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    plot_data()
