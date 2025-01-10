import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
from kalmanFilter import KalmanFilter
from itertools import permutations
import time

def generate_cost_matrix(pred_tracks, active_tracks, past_active_tracks, past_pred_tracks, detections, distance_weight=0.3, direction_weight=1.0):

    """
    Input: -pred_tracks: list of the all the positions of each track in the present, 
           -active_tracks: list of active tracks,
           -past_active_tracks: list of active tracks in the previous iteration,
           -past_pred_tracks: list of all the positions of each track in the previous iteration,
           -detections: list of detections,
           -distance_weight=0.3: weight that the distance will have to the cost,
           -direction_weight=0.3: weight that the direction will have to the cost
    Output: cost_matrix
    """

    n = len(active_tracks)
    m = len(detections)

    # inicializar cost_matrix com 0s
    cost_matrix = np.zeros((n, m))

    for i in range(n):        
        for j in range(m):
            pred_track_pos = pred_tracks[active_tracks[i]]  
            detection_pos = detections[j]
            past_pred_tracks_pos = past_pred_tracks[past_active_tracks[i]]
            
            # calcula a distancia entre as posiçoes atuais das tracks e das deteçoes 
            dx = pred_track_pos[0] - detection_pos[0]
            dy = pred_track_pos[1] - detection_pos[1]
            
            distance = np.sqrt(float(dx)**2 + float(dy)**2)
            ang_dif = 0
            if len(past_active_tracks)>0: # só é possivel calcular a direção se existir deteções passadas
                
                # calcula a direção entre as posiçoes passadas das tracks e das posiçoes presentes das tracks 
                delta_x = past_pred_tracks_pos[0] - pred_track_pos[0]
                delta_y = past_pred_tracks_pos[1] - pred_track_pos[1]

                angle1 = math.atan2(delta_y, delta_x)
                
                # calcula a direção entre as posiçoes autais das tracks e das deteçoes
                delta_x = pred_track_pos[0] - detection_pos[0]
                delta_y = pred_track_pos[1] - detection_pos[1]

                angle2 = math.atan2(delta_y, delta_x)
                
                # calcular a diferença absoluta entre os dois angulos
                ang_dif = np.abs(angle1-angle2)

            cost_matrix[i][j] = round(distance, 2)*distance_weight + round(ang_dif, 2)*direction_weight
    return cost_matrix

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

    return best_assignment


def main():
    # Load detection positions from the files (including confidence)
    # detections = pd.read_csv('car1_detection_pos3.txt')
    #positions = pd.read_csv('positions.txt')
    
    pred_tracks = {}   # map of predicted tracks
    past_pred_tracks = {}   # map of past predicted tracks
    tracks_count_id = 0 # counter of tracks used for id atribution
    active_tracks = []  # list with the ids of active tracks
    past_active_tracks = []   # list with the ids of active tracks in the past iteration
    kf_map = {}          # map with of Kalman filters associated to each track

    # Skip the first line (header)
    detections = detections.iloc[1:]
    #positions = positions.iloc[1:]

    # Sort detections by the timestamp
    # detections = detections.sort_values(by='timestamp')
    # Group detections by timestamp
    # grouped_detections = detections.groupby('timestamp')
    
    timestamp = time.time()

    for key in pred_tracks: #atualiza o array de tracks ativas, removendo as tracks que não sao atualizadas a mais de 5 segundos
        if timestamp - pred_tracks[key][2] > 5 and key in active_tracks:
            active_tracks.remove(key)
            if key in past_active_tracks:
                past_active_tracks.remove(key)
        
    pos = np.array([unique_group['x'], unique_group['y']]).T
        
    if len(active_tracks) == 0: # if we dont have any tracks yet, the detections will be the first element of the new tracks
            
        for i in range(len(pos)):
            pred_tracks[tracks_count_id] = (pos[i, 0], pos[i, 1], timestamp)
                
            active_tracks.append(tracks_count_id)
            kf_map[tracks_count_id] = KalmanFilter(0.05, 0, 0, 0, 0.3, 0.3)

            tracks_count_id += 1
                
    else:
        cost_matrix = generate_cost_matrix(pred_tracks, active_tracks, past_active_tracks, past_pred_tracks, pos)
        min_cost_indices = optimal_assignment(cost_matrix) # we calculate the otimal assignement
        """
        cost matrix:   
                        detections:
                      0    1    3
                0  [[0.8  0.3  0.9],          
        tracks: 1   [0.3  0.9  0.9],
                2   [0.9  0.8  0.3]]

        min_cost_indices: [1  0  2]
        
        track 0 -> detection 1
        track 1 -> detection 0
        track 2 -> detection 2
        """
            
        if len(active_tracks) == len(pos): #number of detections is equal to the number of active tracks

            if len(min_cost_indices) != len(set(min_cost_indices)): 
            # if min_cost_indices has repeated values, that means that a detection was atribuited to more than one track
                min_cost_indices = optimal_assignment(cost_matrix) # we calculate the otimal assignement
                
                #with the cost matrix, we can atribuit each detection to its track
            for i,value in enumerate(active_tracks):
                pred_tracks[value] = (pos[min_cost_indices[i], 0], pos[min_cost_indices[i], 1], timestamp)
                
                     
        elif len(active_tracks) < len(pos): #number of detections is greater than the number of active tracks
                
            # the existing tracks get updated with the new detection
            for i,value in enumerate(active_tracks):
                pred_tracks[value] = (pos[min_cost_indices[i], 0], pos[min_cost_indices[i], 1], timestamp)
 
            # a new track for each detection that doesnt have a track is added
            for i in range(len(pos) - len(active_tracks)):
                pred_tracks[tracks_count_id] = (pos[len(active_tracks)-1+i, 0], pos[len(active_tracks)-1+i, 1], timestamp)
                active_tracks.append(tracks_count_id)
                kf_map[tracks_count_id] = (KalmanFilter(0.05, 0, 0, 0, 0.3, 0.3))

                tracks_count_id += 1  
            
        else: # there are more tracks than detections, meaning that one track is over or hiden
            min_cost = 0
            tracks_aux = -1
            pos_aux = -1
        
            for j in range(len(pos)):
                min_cost = cost_matrix[0][j]
                pos_aux = j
                tracks_aux = 0
                for i in range(len(active_tracks)): # calculates which tracks have the lower cost for the detections
                    if cost_matrix[i][j] < min_cost:
                        min_cost = cost_matrix[i][j]
                        tracks_aux = i 
                            
                pred_tracks[active_tracks[tracks_aux]] = (pos[pos_aux, 0], pos[pos_aux, 1], timestamp)

    # Update each Kalman filter with the current position
    for at in active_tracks:

        prediction = kf_map[at].predict()  # Predict the next position with the KALMAN FILTER
        update = kf_map[at].update([pred_tracks[at][0], pred_tracks[at][1]])  # Update with the current position with the KALMAN FILTER
        
    past_pred_tracks = pred_tracks.copy()
    past_active_tracks = active_tracks.copy()

if __name__ == "__main__":
    main()
