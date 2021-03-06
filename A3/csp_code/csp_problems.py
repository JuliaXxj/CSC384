from csp import Constraint, Variable, CSP
from constraints import *
from backtracking import bt_search
import util


##################################################################
### NQUEENS
##################################################################

def nQueens(n, model):
    '''Return an n-queens CSP, optionally use tableContraints'''
    #your implementation for Question 4 changes this function
    #implement handling of model == 'alldiff'
    if not model in ['table', 'alldiff', 'row']:
        print("Error wrong sudoku model specified {}. Must be one of {}").format(
            model, ['table', 'alldiff', 'row'])

    i = 0
    dom = []
    for i in range(n):
        dom.append(i+1)

    vars = []
    for i in dom:
        vars.append(Variable('Q{}'.format(i), dom))

    cons = []

    if model == 'alldiff':


        for qi in range(len(dom)):
            for qj in range(qi+1, len(dom)):
                con = NeqConstraint("C(Q{},Q{})".format(qi+1,qj+1),[vars[qi], vars[qj]], qi+1, qj+1)
                cons.append(con)

        allDiffCnsName = ""

        if len(vars) > 0:
            allDiffCnsName += vars[0].name()
        for v in vars[1:]:
            allDiffCnsName = allDiffCnsName + "," + v.name()

        allDiffCns = AllDiffConstraint("C({})".format(allDiffCnsName), vars)
        cons.append(allDiffCns)

    else:
        constructor = QueensTableConstraint if model == 'table' else QueensConstraint
        for qi in range(len(dom)):
            for qj in range(qi+1, len(dom)):
                con = constructor("C(Q{},Q{})".format(qi+1,qj+1),
                                            vars[qi], vars[qj], qi+1, qj+1)
                cons.append(con)

    csp = CSP("{}-Queens".format(n), vars, cons)
    return csp

def solve_nQueens(n, algo, allsolns, model='row', variableHeuristic='fixed', trace=False):
    '''Create and solve an nQueens CSP problem. The first
       parameer is 'n' the number of queens in the problem,
       The second specifies the search algorithm to use (one
       of 'BT', 'FC', or 'GAC'), the third specifies if
       all solutions are to be found or just one, variableHeuristic
       specfies how the next variable is to be selected
       'random' at random, 'fixed' in a fixed order, 'mrv'
       minimum remaining values. Finally 'trace' if specified to be
       'True' will generate some output as the search progresses.
    '''
    csp = nQueens(n, model)
    solutions, num_nodes = bt_search(algo, csp, variableHeuristic, allsolns, trace)
    print("Explored {} nodes".format(num_nodes))
    if len(solutions) == 0:
        print("No solutions to {} found".format(csp.name()))
    else:
       print("Solutions to {}:".format(csp.name()))
       i = 0
       for s in solutions:
           i += 1
           print("Solution #{}: ".format(i)),
           for (var,val) in s:
               print("{} = {}, ".format(var.name(),val), end='')
           print("")


##################################################################
### Class Scheduling
##################################################################

NOCLASS='NOCLASS'
LEC='LEC'
TUT='TUT'
class ScheduleProblem:
    '''Class to hold an instance of the class scheduling problem.
       defined by the following data items
       a) A list of courses to take

       b) A list of classes with their course codes, buildings, time slots, class types,
          and sections. It is specified as a string with the following pattern:
          <course_code>-<building>-<time_slot>-<class_type>-<section>

          An example of a class would be: CSC384-BA-10-LEC-01
          Note: Time slot starts from 1. Ensure you don't make off by one error!

       c) A list of buildings

       d) A positive integer N indicating number of time slots

       e) A list of pairs of buildings (b1, b2) such that b1 and b2 are close
          enough for two consecutive classes.

       f) A positive integer K specifying the minimum rest frequency. That is,
          if K = 4, then at least one out of every contiguous sequence of 4
          time slots must be a NOCLASS.

        See class_scheduling.py for examples of the use of this class.
    '''

    def __init__(self, courses, classes, buildings, num_time_slots, connected_buildings,
        min_rest_frequency):
        #do some data checks
        for class_info in classes:
            info = class_info.split('-')
            if info[0] not in courses:
                print("ScheduleProblem Error, classes list contains a non-course", info[0])
            if info[3] not in [LEC, TUT]:
                print("ScheduleProblem Error, classes list contains a non-lecture and non-tutorial", info[1])
            if int(info[2]) > num_time_slots or int(info[2]) <= 0:
                print("ScheduleProblem Error, classes list  contains an invalid class time", info[2])
            if info[1] not in buildings:
                print("ScheduleProblem Error, classes list  contains a non-building", info[3])

        for (b1, b2) in connected_buildings:
            if b1 not in buildings or b2 not in buildings:
                print("ScheduleProblem Error, connected_buildings contains pair with non-building (", b1, ",", b2, ")")

        if num_time_slots <= 0:
            print("ScheduleProblem Error, num_time_slots must be greater than 0")

        if min_rest_frequency <= 0:
            print("ScheduleProblem Error, min_rest_frequency must be greater than 0")

        #assign variables
        self.courses = courses
        self.classes = classes
        self.buildings = buildings
        self.num_time_slots = num_time_slots
        self._connected_buildings = dict()
        self.min_rest_frequency = min_rest_frequency

        #now convert connected_buildings to a dictionary that can be index by building.
        for b in buildings:
            self._connected_buildings.setdefault(b, [b])

        for (b1, b2) in connected_buildings:
            self._connected_buildings[b1].append(b2)
            self._connected_buildings[b2].append(b1)

    #some useful access functions
    def connected_buildings(self, building):
        '''Return list of buildings that are connected from specified building'''
        return self._connected_buildings[building]




def solve_schedules(schedule_problem, algo, allsolns,
                 variableHeuristic='mrv', silent=False, trace=False):
    #Your implementation for Question 6 goes here.
    #
    #Do not but do not change the functions signature
    #(the autograder will twig out if you do).

    #If the silent parameter is set to True
    #you must ensure that you do not execute any print statements
    #in this function.
    #(else the output of the autograder will become confusing).
    #So if you have any debugging print statements make sure you
    #only execute them "if not silent". (The autograder will call
    #this function with silent=True, class_scheduling.py will call
    #this function with silent=False)

    #You can optionally ignore the trace parameter
    #If you implemented tracing in your FC and GAC implementations
    #you can set this argument to True for debugging.
    #
    #Once you have implemented this function you should be able to
    #run class_scheduling.py to solve the test problems (or the autograder).
    #
    #
    '''This function takes a schedule_problem (an instance of ScheduleProblem
       class) as input. It constructs a CSP, solves the CSP with bt_search
       (using the options passed to it), and then from the set of CSP
       solution(s) it constructs a list (of lists) specifying possible schedule(s)
       for the student and returns that list (of lists)

       The required format of the list is:
       L[0], ..., L[N] is the sequence of class (or NOCLASS) assigned to the student.

       In the case of all solutions, we will have a list of lists, where the inner
       element (a possible schedule) follows the format above.
    '''

    #BUILD your CSP here and store it in the varable csp
    #Let each time slot be a variable and classes in the self.classes be the legal values for each variable's constraint
    vars = []
    for i in range(schedule_problem.num_time_slots):
        dom = findDomain(schedule_problem, i)
        vars.append(Variable("TS{}".format(i+1), dom))

    cons = []
    # constraint:  The student takes all the courses on her list and for each course, the student must take both the lecture and tutorial
    #Use NValuesConstraint here
    for c in schedule_problem.courses:
        allLec = findAllPossibleClasses(LEC, c, schedule_problem.classes)
        allTut = findAllPossibleClasses(TUT, c, schedule_problem.classes)
        cons.append(NValuesConstraint( "course {} lecture constraint".format(c),vars, allLec, 1, 1))
        cons.append(NValuesConstraint("course {} tutorial constraint".format(c), vars, allTut, 1, 1))

    # constraint: for each course, the lec must be taken before the tut of that course
    # use TableConstraint
    allPossibleOrders = findAllPossibleOrders(schedule_problem.classes)
    for v1 in range(len(vars)):
        for v2 in range(v1 + 1, len(vars)):
            cons.append(TableConstraint("legal class order on {} and {}".format(vars[v1].name(), vars[v2].name()),
                                        [vars[v1], vars[v2]], allPossibleOrders))

    #constraint: minimum frequency
    # use NValueConstraint
    if schedule_problem.min_rest_frequency < schedule_problem.num_time_slots:
        cons.append(NValuesConstraint("minimum rest frquency constraint",
                                      vars[0:schedule_problem.min_rest_frequency+1], [NOCLASS], 1, schedule_problem.min_rest_frequency+1))
        for v in range(1, len(vars)-schedule_problem.min_rest_frequency - 1):
            cons.append(NValuesConstraint("minimum rest frquency constraint",
                                          vars[v:v+schedule_problem.min_rest_frequency+1], [NOCLASS], 1, schedule_problem.min_rest_frequency+1))

    #constraint: connected building
    # use TableConstraint
    if schedule_problem.num_time_slots > 2:
        allPossibleConnectedClasses = findAllPossibleConnectedClasses(schedule_problem)
        cons.append(TableConstraint("connected class on {} and {}".format(vars[0].name(), vars[1].name()),
                                    vars[0:2], allPossibleConnectedClasses))
        for v in range(1, len(vars)-1):
                cons.append(TableConstraint("legal class order on {} and {}".format(vars[v].name(), vars[v+1].name()),
                                            vars[v:v+2], allPossibleConnectedClasses))

    csp = CSP("class scheduling", vars, cons)

    #invoke search with the passed parameters
    solutions, num_nodes = bt_search(algo, csp, variableHeuristic, allsolns, trace)

    #Convert each solution into a list of lists specifying a schedule
    #for each student in the format described above.

    #then return a list containing all converted solutions
    solns = []
    for sol in solutions:
        sol_conv = [NOCLASS] * schedule_problem.num_time_slots
        for (var, val) in sol:
            ind = int(var.name()[-1]) - 1
            sol_conv[ind] = val
        solns.append(sol_conv)
    return solns


def findDomain(schedule_problem, time_slot):
    """
    Return a list of classes will be held on the time time_slot
    """
    dom = [NOCLASS]
    for clas in schedule_problem.classes:
        course_info = clas.split("-")
        if int(course_info[2]) == (time_slot + 1):
            dom.append(clas)
    return dom


def findAllPossibleClasses(lecOrTut, course, allClasses):
    """
    Return a list of classes whose course code is course and type is lecOrTut
    """

    allPossibleClasses = []
    for c in allClasses:
        course_info = c.split("-")
        if course_info[0] == course and course_info[3] == lecOrTut:
            allPossibleClasses.append(c)
    return allPossibleClasses


def findAllPossibleOrders(allClasses):
    """
    Return a list of lists which are all possible combination of classes that not violate the constraint that
    one course's tut must be taken after the lec of that course
    """
    allPossibleOrders = [[NOCLASS, NOCLASS]]
    for frstClass in allClasses:
        allPossibleOrders.extend([[NOCLASS, frstClass], [frstClass, NOCLASS]])
        frstClass_info = frstClass.split("-")
        for secClass in allClasses:
            secClass_info = secClass.split("-")
            if frstClass_info[0] == secClass_info[0] and frstClass_info[3] == secClass_info[3]:
                continue
            if frstClass_info[0] == secClass_info[0] and frstClass_info[3] == TUT and secClass_info[3] == LEC:
                continue
            allPossibleOrders.append([frstClass, secClass])

    return allPossibleOrders


def findAllPossibleConnectedClasses(schedule_problem):
    """
    Return a list of lists which are all possible combination of classes that not violate the constraint that
    the buildings of two successor classes must be connected.
    """
    allPossibleConnectedClasses = [[NOCLASS, NOCLASS]]
    for c1 in schedule_problem.classes:
        allPossibleConnectedClasses.extend([[NOCLASS, c1], [c1,NOCLASS]])
        c1_info = c1.split("-")
        for c2 in schedule_problem.classes:
            c2_info = c2.split("-")
            if c1 == c2:
                continue
            if c2_info[1] in schedule_problem.connected_buildings(c1_info[1]):
                allPossibleConnectedClasses.append([c1, c2])
    return allPossibleConnectedClasses



