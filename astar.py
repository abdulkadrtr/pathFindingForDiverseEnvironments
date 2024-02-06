import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
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
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
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


def visualize_path(matris, path):
    matris_with_path = np.copy(matris)

    for point in path:
        matris_with_path[point[0], point[1]] = 2  # Assign a unique value for the path

    cmap = ListedColormap(['white', 'black', 'red'])  # 0: white, 1: black, 2: red

    plt.imshow(matris_with_path, cmap=cmap, interpolation='nearest')
    plt.title('Matrix and Path')
    plt.show()

def main():
    matris = np.random.choice([0, 1], size=(10, 10))
    start = (0, 0)
    goal = (5, 5)
    path = astar(matris, start, goal)
    path = path + [start]
    if path:
        print("Path found")
        visualize_path(matris, path)
        print(path)
    else:
        print("No path found")



if __name__ == '__main__':
    main()