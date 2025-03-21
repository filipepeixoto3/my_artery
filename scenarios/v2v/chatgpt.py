import numpy as np
import matplotlib.pyplot as plt
import sys

class KalmanFilter:
    def __init__(self):
        # Initialize state transition matrix (A), observation matrix (H), etc.
        self.A = np.array([[1, 0, 0.5, 0],
                           [0, 1, 0, 0.5],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # Constant velocity model
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # We only observe position (x, y)
        self.P = np.eye(4) * 500  # Initial state covariance matrix
        self.R = np.eye(2) * 0.1  # Measurement noise covariance
        self.Q = np.eye(4) * 0.01  # Process noise covariance
        self.x = np.zeros((4, 1))  # Initial state

    def predict(self):
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[:2]  # Return predicted position (x, y)

    def update(self, z):
        y = z - np.dot(self.H, self.x)  # Innovation
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Innovation covariance
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kalman gain
        self.x = self.x + np.dot(K, y)
        
        I = np.eye(self.H.shape[1])  # Identity matrix
        self.P = (I - np.dot(K, self.H)) * self.P  # Update state covariance

# Function to read measurement data from a file
def read_measurements(file_path):
    measurements = []
    with open(file_path, 'r') as file:
        next(file)  # Skip header row
        for line in file:
            parts = line.strip().split(',')
            if len(parts) >= 4:
                id_, timestamp, x, y = parts
                measurements.append((float(x), float(y)))  # Only take the x, y values
    return measurements

# Main function to process data and apply Kalman filter
def main(file_path):
    # Read measurements from the file
    measurements = read_measurements(file_path)

    kf = KalmanFilter()
    
    predicted_positions = []
    updated_positions = []
    
    for z in measurements:
        predicted = kf.predict()
        predicted_positions.append(predicted)
        kf.update(np.array(z).reshape(2, 1))
        updated_positions.append(kf.x[:2])

    # Convert results to numpy arrays for easier plotting
    predicted_positions = np.array(predicted_positions).reshape(-1, 2)
    updated_positions = np.array(updated_positions).reshape(-1, 2)
    measurements = np.array(measurements).reshape(-1, 2)

    # Plot the results
    plt.plot(measurements[:, 0], measurements[:, 1], 'go', label='Measurements')
    plt.plot(predicted_positions[:, 0], predicted_positions[:, 1], 'r-', label='Predicted')
    plt.plot(updated_positions[:, 0], updated_positions[:, 1], 'b-', label='Updated (Kalman Filter)')
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Kalman Filter Tracking')
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python kalman_filter.py <file_path>")
    else:
        file_path = sys.argv[1]
        main(file_path)
