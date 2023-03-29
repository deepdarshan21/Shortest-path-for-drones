
# Drone Path Finding Algorithm

A algorithm that simulates the path of drone in 2D plane, it tries to ensure that each drone takes minimum time to reach destination and also not collide with each other. 





## Run Locally

To run this project in your computer is very simple you just need any C++ compiler. After that you can clone this repo into your PC and run the `Drone Path Finding.cpp` file with your compiler.

Clone the project

```bash
  git clone https://github.com/deepdarshan21/Shortest-path-for-drones.git
```

Go to the project directory

```bash
  cd Shortest-path-for-drones
```




## Screenshots
Input
![Input](https://github.com/deepdarshan21/Shortest-path-for-drones/blob/main/screenshot/2023-03-29.png?raw=true)

![](https://github.com/deepdarshan21/Shortest-path-for-drones/blob/main/screenshot/2023-03-29%20(1).png?raw=true)

Output
![](https://github.com/deepdarshan21/Shortest-path-for-drones/blob/main/screenshot/2023-03-29%20(2).png?raw=true)

## Project Explain

The  code is an implementation of the A* search algorithm used for finding the shortest path between two points on a 2D grid with an option of considering varying adjacency types with different penalties for collisions. 

- The `Drone` class contains the `start`, `end`, `start_time` and `path` properties. It also has a constructor that accepts `start`, `end`, and `start time` values, and an operator overloading method for `<` that compares the starting time of two drones.
- `get_adjacent_positions` function is used to get all adjacent positions for the given position on the 2D grid based on the given `adjacency_type` i.e either 4 or 8. These adjacent positions are returned as a vector.
- `heuristic` function is used to calculate the heuristic value (Manhattan distance) between a given position and the end position. The calculation is based on the absolute difference between the x and y values of the two positions.
- The `hash_pair` struct is a function object that overrides the call operator and implements the hash function to create a hash value for a pair of template types.
- `a_star` function is an implementation of A* search algorithm with an option of considering 4-adjacency or 8-adjacency. `collision_penalty` is used to assign extra cost to cells that would collide with another drone's path. The function returns a vector of pairs representing the path from the start position to the end position.
- `get_paths` function takes a vector of `Drone` objects, `grid_size`, adjacency_type, and collision_penalty as inputs, and returns a vector of vectors (representing paths for each drone). Active drones are sorted by their starting time, and `a_star` function is invoked to calculate their paths, avoiding the previous paths of all other drones.
- Finally, the `main` function creates a vector of Drone objects named `drones` and then calls the `get_paths()` function to obtain the paths for each drone. The obtained paths are then printed to the console with the position coordinates of each point in the final path.

## 🔗 Links
[![portfolio](https://img.shields.io/badge/my_portfolio-000?style=for-the-badge&logo=ko-fi&logoColor=white)](https://github.com/deepdarshan21)
[![linkedin](https://img.shields.io/badge/linkedin-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/deepdarshan21/)
[![twitter](https://img.shields.io/badge/twitter-1DA1F2?style=for-the-badge&logo=twitter&logoColor=white)](https://twitter.com/deepdarshan21)


## Feedback

If you have any feedback, please reach out to us at deepdarshan21@gmail.com

