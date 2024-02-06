import numpy as np
import heapq
import matplotlib.pyplot as plt

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1,0), (0,-1,0), (1,0,0), (-1,0,0), (0,0,1), (0,0,-1),(1,1,0),(-1,1,0),(-1,-1,0),(1,-1,0),(0,1,1),(0,-1,1),(0,1,-1),(0,-1,-1),(1,0,1),(-1,0,1),(1,0,-1),(-1,0,-1),(1,1,1),(-1,1,1),(-1,-1,1),(1,-1,1),(1,1,-1),(-1,1,-1),(-1,-1,-1),(1,-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j, k in neighbors:
            neighbor = current[0] + i, current[1] + j, current[2] + k            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if 0 <= neighbor[2] < array.shape[2]:                
                        if array[neighbor[0]][neighbor[1]][neighbor[2]] == 1:
                            continue
                    else:
                        # array bound z walls
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

def visualize_3d_map_and_path(map, path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x, y, z = np.nonzero(map)
    ax.scatter(x, y, z, c='blue', marker='o', label='Obstacles')

    if path is not None:
        path = np.array(path).T
        ax.plot(path[0], path[1], path[2], c='red', marker='o', label='Path')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('3D Map and Path')
    plt.legend()
    plt.show()

def main():
    shape = (10, 10, 10)
    map = np.zeros(shape, dtype=int)
    map[5:8, 5:8, 5:8] = 1
    map[2:6, 2:6, 2:6] = 1
    start = (0, 0, 0)
    goal = (9, 9, 9)
    path = astar(map, start, goal)
    print(path)
    visualize_3d_map_and_path(map, path)






if __name__ == '__main__':
    main()