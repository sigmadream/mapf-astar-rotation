import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


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
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    table = dict()
    for constraint in constraints:
        if constraint['agent'] == agent:
            if constraint['timestep'] not in table:
                table[constraint['timestep']] = [constraint]
            else:
                table[constraint['timestep']].append(constraint)
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


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
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


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    open_list = []
    closed_list = dict()
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
    return None


if __name__ == '__main__':
    my_map = [[True, True, True, True, True, True, True], [True, False, False, False, False, False, True],
              [True, True, True, False, True, True, True], [True, True, True, True, True, True, True]]
    starts = [(1, 1), (1, 2)]
    goals = [(1, 5), (1, 4)]
    heuristics = []
    constraints = []
    paths = []

    [heuristics.append(compute_heuristics(my_map, goal)) for goal in goals]

    for agent in range(len(starts)):
        path = a_star(my_map, starts[agent], goals[agent], heuristics[agent], agent, constraints)
        if path is None:
            raise Exception('No solutions')
        paths.append(path)

    print(paths)
