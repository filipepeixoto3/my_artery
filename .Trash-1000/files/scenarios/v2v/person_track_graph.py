import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Load the data from CSV
df = pd.read_csv('1.0_tracks.csv')

# Initialize the plot
fig, ax = plt.subplots()

# Set the color for both gt and track coordinates (same color)
coord_color = (0.6, 0.8, 1)  # light blue color

# Initialize the scatter plots for ground truth and track coordinates
gt_coords_all = []
track_coords_all = []

# Function to initialize the plot
def init():
    ax.set_xlim(df['gt_x'].min() - 1, df['gt_x'].max() + 1)
    ax.set_ylim(df['gt_y'].min() - 1, df['gt_y'].max() + 1)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    return []

# Function to update the plot for each frame
def update(frame):
    timestamp = df.iloc[frame]['timestamp']
    
    # Get the current ground truth and track coordinates
    gt_coords = df[df['timestamp'] == timestamp][['gt_x', 'gt_y']].values[0]
    track_coords = df[df['timestamp'] == timestamp][['track_x', 'track_y']].values[0]
    
    # Add the current coordinates to the lists
    gt_coords_all.append(gt_coords)
    track_coords_all.append(track_coords)
    
    # Clear the previous scatter plots
    ax.clear()
    
    # Redraw the axis and labels
    ax.set_xlim(df['gt_x'].min() - 1, df['gt_x'].max() + 1)
    ax.set_ylim(df['gt_y'].min() - 1, df['gt_y'].max() + 1)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    
    # Plot all past ground truth coordinates as circles (gt)
    ax.scatter(*zip(*gt_coords_all), color=coord_color, marker='o', s=20, label='Ground Truth')
    
    # Plot all past track coordinates as crosses (track)
    ax.scatter(*zip(*track_coords_all), color=coord_color, marker='x', s=40, label='Track')

    # Add the legend
    ax.legend(loc='upper left')
    
    return []

# Create an animation with the given update function and interval based on timestamps
ani = animation.FuncAnimation(fig, update, frames=len(df), init_func=init, blit=False, interval=100)

# Use plt.show() to display the plot
plt.show()







