import numpy as np
from itertools import permutations

# Example cost matrix (number of tracks = number of detections)
cost_matrix = np.array([
    [10, 19, 8],
    [10, 18, 7],
    [13, 16, 9]
])

def optimal_assignment(cost_matrix):
    num_tracks = cost_matrix.shape[0]
    
    # Generate all possible assignments (permutations)
    indices = range(num_tracks)
    best_cost = float('inf')
    best_assignment = None

    for perm in permutations(indices):
        # Calculate the cost for this assignment
        total_cost = sum(cost_matrix[i, perm[i]] for i in indices)
        
        # Check if this is the best cost found
        if total_cost < best_cost:
            best_cost = total_cost
            best_assignment = perm

    return best_assignment, best_cost

# Execute the function
assignment, cost = optimal_assignment(cost_matrix)
print("Optimal Assignment:", assignment)
print("Total Cost:", cost)