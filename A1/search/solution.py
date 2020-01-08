#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions

from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    count = 0
    for box in state.boxes:
        if box in state.storage:
            continue
        x = box[0]
        y = box[1]
        shortest = state.width + state.height
        for st in state.storage:
            d = abs(st[0] - x) + abs(st[1] - y)
            if d < shortest:
                shortest = d
        count += shortest

    return count


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    distance = 0
    storages = []
    robots = []
    storages.extend(state.storage)
    robots.extend(state.robots)
    for box in state.boxes:
        if box in state.storage:
            continue
        if unmoveable(state, box):
            return float('inf')
        x = box[0]
        y = box[1]
        nearest_robot = state.width + state.height
        r = s = float('inf')
        nearest_storage = state.width + state.height
        for i in range(len(robots)):
            robot = robots[i]
            d = abs(robot[0] - x) + abs(robot[1] - y)
            if d < nearest_robot:
                nearest_robot = d
                r = i
        for j in range(len(storages)):
            st = storages[j]
            if storages[j] in state.boxes:
                continue
            d = abs(st[0] - x) + abs(st[1] - y)
            if d < nearest_storage:
                nearest_storage = d
                s = j
        if s < len(storages):
            robots[r] = storages[s]
            storages.pop(s)

        distance = distance + nearest_robot + nearest_storage
    return distance


def unmoveable(state, box):
    ''' Check whether the box is in some place that unable to move.'''

    up_wall = False
    down_wall = False
    left_wall = False
    right_wall = False
    diag_wall = False
    up_box = False
    down_box = False
    left_box = False
    right_box = False
    diag_box = False

    up = (box[0], box[1] - 1)
    down = (box[0], box[1] + 1)
    left = (box[0] - 1, box[1])
    right = (box[0] + 1, box[1])
    diag = (box[0] + 1, box[1] + 1)

    if box[0] == 0 or left in state.obstacles:
        left_wall = True
    elif left in state.boxes:
        left_box = True
    if box[0] == state.width - 1 or right in state.obstacles:
        right_wall = True
    elif right in state.boxes:
        right_box = True
    if box[1] == 0 or down in state.obstacles:
        down_wall = True
    elif down in state.boxes:
        down_box = True
    if box[1] == state.height - 1 or up in state.obstacles:
        up_wall = True
    elif up in state.boxes:
        up_box = True
    if diag in state.boxes:
        diag_box = True
    elif (right_wall or down_wall) or diag in state.obstacles:
        diag_wall = True

    if ((up_wall or down_wall) and (right_wall or left_wall or right_box or left_box))or((right_wall or left_wall) \
       and (up_box or down_box)) or ((down_wall or down_box) and (diag_wall or diag_box) and (right_wall or right_box)):
        return True
    return False


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """


    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    start_time = os.times()[0]
    end_time = start_time + timebound
    se = SearchEngine('custom', 'path')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    goal = se.search(timebound)
    if not goal:
        return False
    time_left = timebound - (os.times()[0] - start_time)
    while os.times()[0] < end_time and weight >= 0:
        weight -= 1
        new_goal = se.search(time_left, (float('inf'), float('inf'), goal.gval))
        if not new_goal:
            return goal
        goal = new_goal
        time_left = timebound - (os.times()[0] - start_time)
    return goal

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    start_time = os.times()[0]
    se = SearchEngine('best_first', 'path')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    goal = se.search(timebound)
    if not goal:
        return False
    time_left = timebound - (os.times()[0] - start_time)
    while time_left > 0:
        new_goal = se.search(time_left, (goal.gval, float('inf'), float('inf')))
        if not new_goal:
            return goal
        goal = new_goal
        time_left = timebound - (os.times()[0] - start_time)
    return goal
