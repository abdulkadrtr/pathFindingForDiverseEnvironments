# pathFindingForDiverseEnvironments
The implementation of the A* pathfinding algorithm encompasses applications for robotic path planning in various environments, including 3D, 2.5D, and different spatial contexts.

## astar.py

This file contains the pure implementation of the A* algorithm. In the context of a matrix consisting of 0s representing pathways and 1s representing obstacles, the algorithm navigates through the matrix to find the optimal path.

`start = (0,0) and goal = (5,5)`

<img src="https://github.com/abdulkadrtr/pathFindingForDiverseEnvironments/assets/87595266/ac115bfd-b971-48e5-9d63-870b736b3107" width="550" height="450" />

## astar-closest-path.py

This file contains a modified implementation of the A* algorithm. In contrast to the standard behavior where the A* algorithm returns `FALSE` when it cannot find a direct path to the goal, this modified version returns the path to the nearest accessible point to the goal.

`start = (0,0) and goal = (9,9)`


<img src="https://github.com/abdulkadrtr/pathFindingForDiverseEnvironments/assets/87595266/53776685-9aac-472e-b1d6-40d7d75b306c" width="550" height="450" />

## astar-2.5d.py


This file contains a modified implementation of the A* algorithm tailored for robotic applications in a 2.5D map, where the matrix values represent height information in a given area. The algorithm introduces a parameter K, representing the maximum allowable height difference between two adjacent cells. In other words, a path is considered viable between two cells if the height difference between them is less than or equal to the specified K value. This modification is specifically designed for navigating a 2.5D environment, where the algorithm adapts to the terrain's vertical variations while considering the user-defined threshold K.

`start = (0,0) and goal = (4,8)`
![Figure_2](https://github.com/abdulkadrtr/pathFindingForDiverseEnvironments/assets/87595266/fd54c774-dc36-4fcb-8a7a-1aade974dc6c)
![Figure_3](https://github.com/abdulkadrtr/pathFindingForDiverseEnvironments/assets/87595266/7f3d8311-1404-4a73-b8e5-f18a61d82e31)
