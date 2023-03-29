#include <bits/stdc++.h>
using namespace std;

// Define a class for the Drone object
class Drone {
   public:
    pair<int, int> start;         // Starting position of the drone
    pair<int, int> end;           // Ending position of the drone
    int start_time;               // Time when the drone is scheduled to start
    vector<pair<int, int>> path;  // Path taken by the drone

    // Constructor for the Drone object
    Drone() {
        start = make_pair(0, 0);
        end = make_pair(0, 0);
        start_time = 0;
    }
    Drone(pair<int, int> start, pair<int, int> end, int start_time) {
        this->start = start;
        this->end = end;
        this->start_time = start_time;
    }

    // Operator overloading for comparing two Drone objects based on start_time
    bool operator<(const Drone& other) const {
        return start_time < other.start_time;
    }
};

// Custom hash function for pair<int, int> to be used as key in unordered_map
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

// Get adjacent positions based on adjacency_type
vector<pair<int, int>> get_adjacent_positions(pair<int, int> position, int adjacency_type) {
    int x = position.first;
    int y = position.second;
    vector<pair<int, int>> result;

    // If adjacency_type is 8, consider all eight adjacent positions
    if (adjacency_type == 8) {
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i != 0 || j != 0) {
                    result.push_back(make_pair(x + i, y + j));
                }
            }
        }
    }

    // If adjacency_type is 4, consider only the four cardinal adjacent positions
    else if (adjacency_type == 4) {
        result.push_back(make_pair(x + 1, y));
        result.push_back(make_pair(x - 1, y));
        result.push_back(make_pair(x, y + 1));
        result.push_back(make_pair(x, y - 1));
    }
    return result;
}

/*
 * Heuristic function for A* algorithm
 * @param position The current position
 * @param end      The ending position
 */
int heuristic(pair<int, int> position, pair<int, int> end) {
    return abs(position.first - end.first) + abs(position.second - end.second);
}

/*
 * A* algorithm to find the shortest path from start to end in a grid with obstacles
 * @param start             The starting position
 * @param end               The ending position
 * @param grid              The grid with obstacles
 * @param adjacency_type    The type of adjacency for A* algorithm. Defaults to 8
 * @param collision_penalty The penalty for colliding with an obstacle. Defaults to 100
 */
vector<pair<int, int>> a_star(pair<int, int> start, pair<int, int> end, vector<vector<int>> grid, int adjacency_type, int collision_penalty) {
    // Initialize priority queue with start node and its cost
    priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> pq;
    pq.push(make_pair(0, start));

    // Initialize hash maps to keep track of the cost and the parent of each node
    unordered_map<pair<int, int>, pair<int, int>, hash_pair> parent_map;
    unordered_map<pair<int, int>, int, hash_pair> cost_map;
    cost_map[start] = 0;

    while (!pq.empty()) {
        // Pop the node with the lowest cost so far
        pair<int, pair<int, int>> current = pq.top();
        pq.pop();

        // If the current node is the end node, stop searching
        if (current.second == end) {
            break;
        }

        // Get the neighbors of the current node
        vector<pair<int, int>> neighbors = get_adjacent_positions(current.second, adjacency_type);
        for (pair<int, int> next_pos : neighbors) {
            // Check if the neighbor is within the grid boundaries
            if (!(0 <= next_pos.first && next_pos.first < grid.size() && 0 <= next_pos.second && next_pos.second < grid[0].size())) {
                continue;
            }

            int cost;
            // Calculate the cost to move to the neighbor
            if (grid[next_pos.first][next_pos.second] != -1 && grid[next_pos.first][next_pos.second] <= cost_map[current.second]) {
                cost = cost_map[current.second] + collision_penalty;
            } else {
                cost = cost_map[current.second] + 1;
            }

            // Update the cost and the parent of the neighbor if it's cheaper to move to the neighbor through the current node
            if (cost_map.find(next_pos) == cost_map.end() || cost < cost_map[next_pos]) {
                cost_map[next_pos] = cost;
                int priority = cost + heuristic(next_pos, end);
                pq.push(make_pair(priority, next_pos));
                parent_map[next_pos] = current.second;
            }
        }

        // Update the grid with the cost of the current node
        grid[current.second.first][current.second.second] = cost_map[current.second];
    }

    // Reconstruct the path from start to end using the parent map
    vector<pair<int, int>> path;
    path.push_back(end);
    pair<int, int> current = end;
    while (current != start) {
        current = parent_map[current];
        path.push_back(current);
    }
    reverse(path.begin(), path.end());

    return path;
}

/**
 * Computes paths for given drones using A* algorithm
 *
 * @param drones        A vector of drones to compute paths for
 * @param grid_size     A pair representing the size of the grid
 * @param adjacency_type The type of adjacency for A* algorithm. Defaults to 4
 * @param collision_penalty The penalty for collision while computing A* algorithm. Defaults to 100
 *
 * @return A vector of vectors, where each sub-vector represents the path for a drone
 */

vector<vector<pair<int, int>>> get_paths(vector<Drone> drones, pair<int, int> grid_size, int adjacency_type = 4, int collision_penalty = 100) {
    // Create a grid with default value of -1
    vector<vector<int>> grid(grid_size.first, vector<int>(grid_size.second, -1));

    // Create a vector of active drones sorted by their start time
    vector<pair<int, Drone>> active_drones;
    for (Drone drone : drones) {
        active_drones.push_back(make_pair(drone.start_time, drone));
    }
    sort(active_drones.begin(), active_drones.end());

    // Create a vector to store the result paths
    vector<vector<pair<int, int>>> result;

    // Compute paths for each active drone
    for (pair<int, Drone> active_drone : active_drones) {
        // Get the time step and drone
        int time_step = active_drone.first;
        Drone drone = active_drone.second;

        // Compute path using A* algorithm
        vector<pair<int, int>> path = a_star(drone.start, drone.end, grid, adjacency_type, collision_penalty);

        // Update the grid with the drone's path
        for (pair<int, int> pos : path) {
            grid[pos.first][pos.second] = time_step;
        }

        // Store the computed path in the drone object
        drone.path = path;

        // Add the path to the result vector
        result.push_back(path);
    }

    // Return the result vector
    return result;
}

// Example usage:
int main() {
    pair<int, int> grid_size = make_pair(20, 20);
    int num_drones;
    cout << "Enter the number of drones in 2D space: ";
    cin >> num_drones;
    vector<Drone> drones(num_drones);
    for (int i = 0; i < num_drones; i++) {
        int start_time, start_x, start_y, end_x, end_y;
        cout << "Enter the details of drone " << i + 1 << ":-" << endl;
        cout << "   Enter the start time(t): ";
        cin >> start_time;
        cout << "   Enter the start position(x, y): ";
        cin >> start_x >> start_y;
        cout << "   Enter the end position(x, y): ";
        cin >> end_x >> end_y;
        drones[i] = Drone(make_pair(start_x, start_y), make_pair(end_x, end_y), start_time);
        cout << "Congratulations! You have entered the details of drone " << i + 1 << " successfully." << endl;
        cout << "=============================================================================" << endl;
    }
    int adjacency_type;
    cout << "Enter the adjacency type for A* algorithm (4 or 8): ";
    cin >> adjacency_type;

    cout << "Press any key to continue..." << endl;
    getchar();
    system("cls");

    cout << "Computing paths for the given drones..." << endl;

    vector<vector<pair<int, int>>> paths = get_paths(drones, grid_size, adjacency_type);

    cout << "Paths computed successfully!" << endl;
    system("cls");

    vector<vector<string>> grid(grid_size.first, vector<string>(grid_size.second, "_"));
    for (int i = 0; i < num_drones; i++) {
        for (pair<int, int> pos : paths[i]) {
            grid[pos.first][pos.second] = to_string(i + 1);
        }
    }

    cout << "The grid with the paths of the drones is as follows:-" << endl;
    for (int i = 0; i < grid_size.first; i++) {
        for (int j = 0; j < grid_size.second; j++) {
            cout << grid[i][j] << " ";
        }
        cout << endl;
    }

    getchar();
    return 0;
}
