from heapq import heappop, heappush


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]


def check_direction(num1, num2):
    if num1 >= 0 and num2 >= 0:
        return [0, 1]
    elif num1 >= 0 and num2 < 0:
        return [1, 2]
    elif num1 < 0 and num2 < 0:
        return [2, 3]
    elif num1 < 0 and num2 >= 0:
        return [3, 0]


def compute_heuristics(start, goal):
    value = (goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2
    start_direction = start[2]  # 0:up, 1:right, 2:down, 3:left
    start_vector = {0: [-1, 0], 1: [0, 1], 2: [1, 0], 3: [0, -1]}
    goal_direction = check_direction(int(goal[0] > start[0]), int(goal[1] > start[1]))
    inner_product = start_vector[start_direction][0] * goal_direction[0] + start_vector[start_direction][1] * \
                    goal_direction[1]
    if inner_product > 0:
        return value
    else:
        return value + 1


def a_star(my_map, start_loc, goal_loc):
    neighbors = [(0, 0, 0),
                 (0, 0, 1),
                 (0, 0, 2),
                 (0, 0, 3),
                 (0, 1, 1),
                 (0, -1, 3),
                 (1, 0, 2),
                 (-1, 0, 0)]

    open_list = []
    closed_set = set()
    came_from = {}
    gscore = {start_loc: 0}
    fscore = {start_loc: compute_heuristics(start_loc, goal_loc)}
    heappush(open_list, (fscore[start_loc], start_loc))

    while len(open_list) > 0:
        current = heappop(open_list)[1]
        if current == goal_loc:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        closed_set.add(current)

        for i, j, k in neighbors:
            if abs(current[2] - k) == 2 or abs(current[2] - k) + abs(i) == 2 or abs(current[2] - k) + abs(j) == 2:
                continue
            else:
                neighbor = current[0] + i, current[1] + j, k
            tentative_g_score = gscore[current] + compute_heuristics(current, neighbor)
            if 0 <= neighbor[0] < len(my_map):
                if 0 <= neighbor[1] < len(my_map[0]):
                    if my_map[neighbor[0]][neighbor[1]]:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in closed_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_list]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + compute_heuristics(neighbor, goal_loc)
                heappush(open_list, (fscore[neighbor], neighbor))
    return None


if __name__ == '__main__':
    my_map = [[True, True, True, True, True, True, True], [True, False, False, False, False, False, True],
              [True, True, True, False, True, True, True], [True, True, True, True, True, True, True]]
    starts = [(1, 1, 1), (1, 2, 1)]
    goals = [(1, 5, 1), (1, 4, 1)]
    heuristics = []
    constraints = []
    paths = []

    for agent in range(len(starts)):
        path = a_star(my_map, starts[agent], goals[agent])
        if path is None:
            raise Exception('No solutions')
        paths.append(path)
    print(paths)
