import random

import problems
import search
from game import Agent
from game import Directions
import time

class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class SearchAgent(Agent):
    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        # TODO 11
        if self.searchFunction == None: raise (Exception, "No search function provided for SearchAgent")
        starttime = time.time()
        problem = self.searchType(state)
        self.actions = self.searchFunction(problem)
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)
    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        # TODO 12
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class BFSFoodSearchAgent(SearchAgent):
    # TODO 13
    def __init__(self):
        self.searchFunction = lambda prob: search.breadthFirstSearch(prob)
        self.searchType = problems.SingleFoodSearchProblem


class DFSFoodSearchAgent(SearchAgent):
    # TODO 14
    def __init__(self):
        self.searchFunction = lambda prob: search.depthFirstSearch(prob)
        self.searchType = problems.MultiFoodSearchProblem


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    def __init__(self):
        self.searchFunction = lambda prob: search.uniformCostSearch(prob)
        self.searchType = problems.MultiFoodSearchProblem


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    pass
