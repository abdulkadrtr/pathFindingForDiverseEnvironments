import numpy as np
import heapq
import matplotlib.pyplot as plt

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal, K):
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
                    if abs(array[current[0]][current[1]] - array[neighbor[0]][neighbor[1]]) > K:
                        continue
                else:
                    continue
            else:
                continue                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

def visualize_path(array, path):
    array = np.copy(array)
    for point in path:
        array[point[0]][point[1]] = 50

    plt.imshow(array, cmap='hot', interpolation='nearest')
    plt.title('2.5D A* Map and Path')
    plt.show()

def main():
    matrix = np.array([
        [1, 2, 3, 4, 5, 7, 7, 8, 9],
        [9, 8, 5, 6, 6, 8, 3, 2, 1],
        [1, 2, 3, 4, 5, 9, 7, 8, 9],
        [9, 8, 4, 6, 8, 9, 3, 2, 1],
        [1, 2, 3, 4, 5, 6, 7, 8, 9]
    ])
    start = (0, 0)
    goal = (4, 8)
    K = 1.0
    path = astar(matrix, start, goal, K)
    path.append(start)
    if path:
        print("Path found")
        print(path)
        visualize_path(matrix, path)
    else:
        print("Path not found")



if __name__ == "__main__":
    main()