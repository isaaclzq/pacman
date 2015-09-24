# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from collections import deque
import util
import sys
import copy
import heapq

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def goalTest(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
        Given a state, returns available actions.
        Returns a list of actions
        """        
        util.raiseNotDefined()

    def getResult(self, state, action):
        """
        Given a state and an action, returns resulting state.
        """
        util.raiseNotDefined()

    def getCost(self, state, action):
        """
        Given a state and an action, returns step cost, which is the incremental cost 
        of moving to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    You are not required to implement this, but you may find it useful for Q5.
    """
    "*** YOUR CODE HERE ***"
    frontier, visited, solution, move = deque(), [], [], 0
    frontier.append([problem.getStartState(), solution, move])
    while frontier:
        node = frontier.popleft()
        curState, curSol, curMove = node[0], node[1], node[2]
        if curMove == 0:
            newState = curState
        else:
            newState = problem.getResult(curState, curMove)
            if problem.goalTest(newState):
                curSol.append(curMove)
                return curSol[1:]
        avaAction = problem.getActions(newState)
        visited.append(newState)
        frontSet = []
        if frontier:
            for front in frontier:
                frontSet.append(problem.getResult(front[0], front[2]))
        for action in avaAction:
            tmpSol = list(curSol)
            tmpSol.append(curMove)
            if problem.goalTest(problem.getResult(newState, action)):
                curSol.append(curMove)
                curSol.append(action)
                return curSol[1:]
            if (problem.getResult(newState, action) not in frontSet) and (problem.getResult(newState, action) not in visited):
                frontier.append([newState, tmpSol, action])
    return False

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
        


def depth_limited_search(problem, depth):
    frontier, visited, solution, level, move = [], [], [], 1, 0
    frontier.append([problem.getStartState(), level, solution, move])
    while frontier:
        node = frontier.pop()
        curState, curLevel, curSol, curMove = node[0], node[1], node[2], node[3]
        if curMove == 0:
            newState = curState
        else:
            newState = problem.getResult(curState, curMove)
            if problem.goalTest(newState):
                curSol.append(curMove)
                return curSol[1:]
        avaAction = problem.getActions(newState)
        visited.append(newState)
        frontSet = []
        if frontier:
            for front in frontier:
                frontSet.append(problem.getResult(front[0], front[3]))
        for action in avaAction:
            tmpSol = list(curSol)
            tmpSol.append(curMove)
            if problem.goalTest(problem.getResult(newState, action)):
                curSol.append(curMove)
                curSol.append(action)
                return curSol[1:]
            if (problem.getResult(newState, action) not in frontSet) and (problem.getResult(newState, action) not in visited) and curLevel < depth:
                frontier.append([newState, curLevel+1, tmpSol, action])
    return False


def iterativeDeepeningSearch(problem):
    """
    Perform DFS with increasingly larger depth.

    Begin with a depth of 1 and increment depth by 1 at every step.
    """
    "*** YOUR CODE HERE ***"
    for depth in range(1000)[1:]:
        sol = depth_limited_search(problem, depth)
        if sol:
            return sol



def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    frontier, visited, solution, move, cost = [], [], [], None, 0
    heapq.heappush(frontier, [heuristic(problem.getStartState(), problem), problem.getStartState(), solution, move, cost])
    while frontier:
        node = heapq.heappop(frontier)
        curH, curState, curSol, curMove, curCost = node[0], node[1], node[2], node[3], node[4]
        if curMove == None:
            newState = curState
        else:
            newState = problem.getResult(curState, curMove)
            if newState in visited:
                continue
            if problem.goalTest(newState):
                curSol.append(curMove)
                return curSol[1:]
        avaAction = problem.getActions(newState)
        visited.append(newState)
        for action in avaAction:
            tmpSol = list(curSol)
            tmpSol.append(curMove)  
            if curMove != None:
                tmpState = problem.getResult(newState, action)
                heapq.heappush(frontier, [heuristic(problem.getResult(newState, action), problem)+curCost+problem.getCost(curState, curMove)+problem.getCost(newState, action), newState, tmpSol, action, curCost+problem.getCost(curState, curMove)])
            else:
                heapq.heappush(frontier, [heuristic(problem.getResult(newState, action), problem)+problem.getCost(newState, action), newState, tmpSol, action, curCost])
    return False

# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
