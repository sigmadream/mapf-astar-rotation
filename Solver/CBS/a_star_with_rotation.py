import heapq
from heapq import heappush


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = dict()
    for constraint in constraints:
        if constraint['agent'] == agent:
            if constraint['timestep'] not in table:
                table[constraint['timestep']] = [constraint]
            else:
                table[constraint['timestep']].append(constraint)
        # task 4
        if constraint['agent'] != agent and constraint['positive'] == True:
            if len(constraint['loc']) > 1:
                cons_i = {'agent': agent,
                          'loc': [constraint['loc'][1], constraint['loc'][0]],
                          'timestep': constraint['timestep'],
                          'positive': False
                          }

            else:
                cons_i = {'agent': agent,
                          'loc': constraint['loc'],
                          'timestep': constraint['timestep'],
                          'positive': False
                          }
            if cons_i['timestep'] not in table:
                table[cons_i['timestep']] = [cons_i]
            else:
                table[cons_i['timestep']].append(cons_i)

    return table
    # pass


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        for constraint in constraint_table[next_time]:
            if len(constraint['loc']) == 1:
                if constraint['loc'] == [next_loc]:
                    if constraint['positive'] == True:
                        return 1
                    else:
                        return 0
            else:
                if constraint['loc'] == [curr_loc, next_loc]:
                    if constraint['positive'] == True:
                        return 1
                    else:
                        return 0
    return -1
    pass


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_loc:
            no_future_goalConstraint = True
            for timestep in table:
                if timestep > curr['timestep']:
                    for cons in table[timestep]:
                        if cons['loc'] == [goal_loc] and cons['positive'] == False:
                            no_future_goalConstraint = False
            if no_future_goalConstraint:
                return get_path(curr)

        continue_flag = False
        for d in range(5):
            child_loc = move(curr['loc'], d)
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, table) == 1:
                child = {'loc': child_loc,
                         'g_val': curr['g_val'] + 1,
                         'h_val': h_values[child_loc],
                         'parent': curr,
                         'timestep': curr['timestep'] + 1
                         }
                if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(
                        my_map[0]):
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
                continue_flag = True
                break

        if continue_flag:
            continue

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1
                     }

            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, table) == 0:
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    return None  # Failed to find solutions


def astar_rotation(array, start, goal):
    def check_direction(num1, num2):
        if num1 >= 0 and num2 >= 0:
            return [0, 1]
        elif num1 >= 0 and num2 < 0:
            return [1, 2]
        elif num1 < 0 and num2 < 0:
            return [2, 3]
        elif num1 < 0 and num2 >= 0:
            return [3, 0]

    def heuristic(a, b):
        value = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        start_direction = a[2]  # 0:up, 1:right, 2:down, 3:left
        start_vector = {0: [-1, 0], 1: [0, 1], 2: [1, 0], 3: [0, -1]}
        goal_direction = check_direction(int(b[0] > a[0]), int(b[1] > a[1]))
        inner_product = start_vector[start_direction][0] * goal_direction[0] + start_vector[start_direction][1] * \
                        goal_direction[1]
        if inner_product > 0:
            return value
        else:
            return value + 1

    neighbors = [(0, 0, 0),
                 (0, 0, 1),
                 (0, 0, 2),
                 (0, 0, 3),
                 (0, 1, 1),
                 (0, -1, 3),
                 (1, 0, 2),
                 (-1, 0, 0)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j, k in neighbors:
            if abs(current[2] - k) == 2 or abs(current[2] - k) + abs(i) == 2 or abs(current[2] - k) + abs(j) == 2:
                continue
            else:
                neighbor = current[0] + i, current[1] + j, k
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False
