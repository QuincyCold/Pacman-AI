"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions
import util

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 17
    frontier = util.Stack()
    exploredNodes = []
    startState = problem.getStartState()
    startNode = (startState, [])

    frontier.push(startNode)

    while not frontier.isEmpty():
        currentState, actions = frontier.pop()

        if currentState not in exploredNodes:
            exploredNodes.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newNode = (succState, newAction)
                    frontier.push(newNode)

    return actions


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18
    frontier = util.Queue()

    exploredNodes = []

    startState = problem.getStartState()
    startNode = (startState, [], 0)

    frontier.push(startNode)

    while not frontier.isEmpty():
        currentState, actions, currentCost = frontier.pop()

        if currentState not in exploredNodes:
            exploredNodes.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)

                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)

                    frontier.push(newNode)

    return actions


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19
    frontier = util.PriorityQueue()

    exploredNodes = {}

    startState = problem.getStartState()
    startNode = (startState, [], 0)

    frontier.push(startNode, 0)

    while not frontier.isEmpty():
        currentState, actions, currentCost = frontier.pop()

        if (currentState not in exploredNodes) or (currentCost < exploredNodes[currentState]):
            exploredNodes[currentState] = currentCost

            if problem.isGoalState(currentState):
                return actions
            else:
                successors = problem.getSuccessors(currentState)

                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)

                    frontier.update(newNode, newCost)

    return actions


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def singleFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    pass


def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    pass


def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22
    frontier = util.PriorityQueue()

    exploredNodes = []

    startState = problem.getStartState()
    startNode = (startState, [], 0)

    frontier.push(startNode, 0)

    while not frontier.isEmpty():

        currentState, actions, currentCost = frontier.pop()

        currentNode = (currentState, currentCost)
        exploredNodes.append((currentState, currentCost))

        if problem.isGoalState(currentState):
            return actions

        else:
            successors = problem.getSuccessors(currentState)

            for succState, succAction, succCost in successors:
                newAction = actions + [succAction]
                newCost = problem.getCostOfActions(newAction)
                newNode = (succState, newAction, newCost)

                already_explored = False
                for explored in exploredNodes:
                    exploredState, exploredCost = explored

                    if (succState == exploredState) and (newCost >= exploredCost):
                        already_explored = True

                if not already_explored:
                    frontier.push(newNode, newCost + heuristic(succState, problem))
                    exploredNodes.append((succState, newCost))

    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
