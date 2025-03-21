#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <set>
#include <algorithm>
#include <ctime>
#include <iterator>
#include <numeric>
#include <limits>
#include <tuple>
#include <array>
#include <functional>

#include "KalmanFilter.cc"


std::vector<std::vector<double>> generate_cost_matrix(
    const std::map<int, std::tuple<double, double, double>>& pred_tracks,
    const std::vector<int>& active_tracks,
    const std::vector<int>& past_active_tracks,
    const std::map<int, std::tuple<double, double, double>>& past_pred_tracks,
    const std::vector<std::array<double, 2>>& detections,
    double distance_weight = 0.3,
    double direction_weight = 1.0) {

    size_t n = active_tracks.size();
    size_t m = detections.size();

    std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(m, 0.0));

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            auto pred_track_pos = pred_tracks.at(active_tracks[i]);
            auto detection_pos = detections[j];
            auto past_pred_tracks_pos = past_pred_tracks.at(past_active_tracks[i]);

            double dx = std::get<0>(pred_track_pos) - detection_pos[0];
            double dy = std::get<1>(pred_track_pos) - detection_pos[1];

            double distance = std::sqrt(dx * dx + dy * dy);
            double ang_dif = 0.0;

            if (!past_active_tracks.empty()) {
                double delta_x = std::get<0>(past_pred_tracks_pos) - std::get<0>(pred_track_pos);
                double delta_y = std::get<1>(past_pred_tracks_pos) - std::get<1>(pred_track_pos);

                double angle1 = std::atan2(delta_y, delta_x);

                delta_x = std::get<0>(pred_track_pos) - detection_pos[0];
                delta_y = std::get<1>(pred_track_pos) - detection_pos[1];

                double angle2 = std::atan2(delta_y, delta_x);

                ang_dif = std::abs(angle1 - angle2);
            }

            cost_matrix[i][j] = std::round(distance * 100.0) / 100.0 * distance_weight + std::round(ang_dif * 100.0) / 100.0 * direction_weight;
        }
    }
    return cost_matrix;
}

std::vector<int> optimal_assignment(const std::vector<std::vector<double>>& cost_matrix) {
    size_t num_tracks = cost_matrix.size();
    std::vector<int> indices(num_tracks);
    std::iota(indices.begin(), indices.end(), 0);
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_assignment;

    do {
        double total_cost = 0.0;
        for (size_t i = 0; i < num_tracks; ++i) {
            total_cost += cost_matrix[i][indices[i]];
        }

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_assignment = indices;
        }
    } while (std::next_permutation(indices.begin(), indices.end()));

    return best_assignment;
}


int main() {
    std::map<int, std::tuple<double, double, double>> pred_tracks;
    std::map<int, std::tuple<double, double, double>> past_pred_tracks;
    std::map<int, int> pred_tracks_repeat;
    int tracks_count_id = 0;
    std::vector<int> active_tracks;
    std::vector<int> past_active_tracks;
    std::map<int, KalmanFilter> kf_map;

    std::vector<std::array<double, 2>> detections; // Load detection positions

    std::time_t timestamp = std::time(nullptr);

    for (auto key : pred_tracks) {
        if (timestamp - std::get<2>(key.second) > 5 && std::find(active_tracks.begin(), active_tracks.end(), key.first) != active_tracks.end()) {
            active_tracks.erase(std::remove(active_tracks.begin(), active_tracks.end(), key.first), active_tracks.end());
            if (std::find(past_active_tracks.begin(), past_active_tracks.end(), key.first) != past_active_tracks.end()) {
                past_active_tracks.erase(std::remove(past_active_tracks.begin(), past_active_tracks.end(), key.first), past_active_tracks.end());
            }
        }
    }

    std::vector<std::array<double, 2>> pos; // Populate with unique_group['x'], unique_group['y']

    if (active_tracks.empty()) {
        for (size_t i = 0; i < pos.size(); ++i) {
            pred_tracks[tracks_count_id] = std::make_tuple(pos[i][0], pos[i][1], timestamp);
            pred_tracks_repeat[tracks_count_id] = 1;

            active_tracks.push_back(tracks_count_id);
            kf_map[tracks_count_id] = KalmanFilter(0.05, 0, 0, 0, 0.3, 0.3);

            tracks_count_id++;
        }
    } else {
        auto cost_matrix = generate_cost_matrix(pred_tracks, active_tracks, past_active_tracks, past_pred_tracks, pos);
        auto min_cost_indices = optimal_assignment(cost_matrix);

        if (active_tracks.size() == pos.size()) {
            if (min_cost_indices.size() != std::set<int>(min_cost_indices.begin(), min_cost_indices.end()).size()) {
                min_cost_indices = optimal_assignment(cost_matrix);
            }

            for (size_t i = 0; i < active_tracks.size(); ++i) {
                pred_tracks[active_tracks[i]] = std::make_tuple(pos[min_cost_indices[i]][0], pos[min_cost_indices[i]][1], timestamp);

                if (pred_tracks_repeat.find(active_tracks[i]) != pred_tracks_repeat.end()) {
                    pred_tracks_repeat[active_tracks[i]]++;
                } else {
                    pred_tracks_repeat[active_tracks[i]] = 1;
                }
            }
        } else if (active_tracks.size() < pos.size()) {
            for (size_t i = 0; i < active_tracks.size(); ++i) {
                pred_tracks[active_tracks[i]] = std::make_tuple(pos[min_cost_indices[i]][0], pos[min_cost_indices[i]][1], timestamp);
                pred_tracks_repeat[active_tracks[i]]++;
            }

            for (size_t i = 0; i < pos.size() - active_tracks.size(); ++i) {
                pred_tracks[tracks_count_id] = std::make_tuple(pos[active_tracks.size() - 1 + i][0], pos[active_tracks.size() - 1 + i][1], timestamp);
                pred_tracks_repeat[tracks_count_id] = 1;
                active_tracks.push_back(tracks_count_id);
                kf_map[tracks_count_id] = KalmanFilter(0.05, 0, 0, 0, 0.3, 0.3);

                tracks_count_id++;
            }
        } else {
            double min_cost = 0;
            int tracks_aux = -1;
            int pos_aux = -1;

            for (size_t j = 0; j < pos.size(); ++j) {
                min_cost = cost_matrix[0][j];
                pos_aux = j;
                tracks_aux = 0;
                for (size_t i = 0; i < active_tracks.size(); ++i) {
                    if (cost_matrix[i][j] < min_cost) {
                        min_cost = cost_matrix[i][j];
                        tracks_aux = i;
                    }
                }

                pred_tracks[active_tracks[tracks_aux]] = std::make_tuple(pos[pos_aux][0], pos[pos_aux][1], timestamp);
                pred_tracks_repeat[active_tracks[tracks_aux]]++;
            }
        }
    }

    for (const auto& at : active_tracks) {
        auto prediction = kf_map[at].predict();
        kf_map[at].update({std::get<0>(pred_tracks[at]), std::get<1>(pred_tracks[at])});
    }

    past_pred_tracks = pred_tracks;
    past_active_tracks = active_tracks;
    return 0;
}