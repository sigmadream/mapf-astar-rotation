import heapq
import time as timer

from .a_star_with_rotation import a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    t_range = max(len(path1), len(path2))
    for t in range(t_range):
        loc_c1 = get_location(path1, t)
        loc_c2 = get_location(path2, t)
        loc1 = get_location(path1, t + 1)
        loc2 = get_location(path2, t + 1)
        if loc1 == loc2:
            return [loc1], t
        if [loc_c1, loc1] == [loc2, loc_c2]:
            return [loc2, loc_c2], t
    return None


def detect_collisions(paths):
    collisions = []
    for i in range(len(paths) - 1):
        for j in range(i + 1, len(paths)):
            if detect_collision(paths[i], paths[j]) is not None:
                position, t = detect_collision(paths[i], paths[j])
                collisions.append({'a1': i,
                                   'a2': j,
                                   'loc': position,
                                   'timestep': t + 1})
    return collisions


def standard_splitting(collision):
    constraints = []
    if len(collision['loc']) == 1:
        constraints.append({'agent': collision['a1'],
                            'loc': collision['loc'],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
        constraints.append({'agent': collision['a2'],
                            'loc': collision['loc'],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
    else:
        constraints.append({'agent': collision['a1'],
                            'loc': [collision['loc'][0], collision['loc'][1]],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
        constraints.append({'agent': collision['a2'],
                            'loc': [collision['loc'][1], collision['loc'][0]],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
    return constraints


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSRSolver(object):
    def __init__(self, my_map, starts, goals):

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        self.start_time = timer.time()

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # print(root['collisions'])

        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        while len(self.open_list) > 0:
            p = self.pop_node()
            if p['collisions'] == []:
                self.print_results(p)
                return p['paths']
            collision = p['collisions'].pop(0)
            constraints = standard_splitting(collision)
            for constraint in constraints:
                q = {'cost': 0,
                     'constraints': [constraint],
                     'paths': [],
                     'collisions': []
                     }
                for c in p['constraints']:
                    if c not in q['constraints']:
                        q['constraints'].append(c)
                for pa in p['paths']:
                    q['paths'].append(pa)

                ai = constraint['agent']
                path = a_star(self.my_map, self.starts[ai], self.goals[ai])
                if path is not None:
                    q['paths'][ai] = path
                    continue_flag = False
                    if constraint['positive']:
                        vol = paths_violate_constraint(constraint, q['paths'])
                        for v in vol:
                            path_v = a_star(self.my_map, self.starts[v], self.goals[v])
                            if len(path_v) == 1:
                                path_v = path_v[0]
                            if path_v is None:
                                continue_flag = True
                            else:
                                q['paths'][v] = path_v
                        if continue_flag:
                            continue
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
