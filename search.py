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

import util
import sys
import copy
import Queue

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
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
        

def DFS(problem, depth):
    frontier = []
    visited = []
    solution = []
    level = 0
    frontier.append([problem.getStartState(), level, solution])
    def getState(node):
        return node[0]
    def getDepth(node):
        return node[1]
    def getSolution(node):
        return node[2]
    while frontier:
        node = frontier.pop()
        curState = getState(node)
        curLevel = getDepth(node)
        curSol = getSolution(node)
        if problem.goalTest(curState):
            return curSol
        else:
            visited.append(curState)
            avaAction = problem.getActions(curState)
            frontSet = []
            if frontier:
                for front in frontier:
                    frontSet.append(getState(front))
            for action in avaAction:
                tmpSol = list(curSol)
                newState = problem.getResult(curState, action)
                if (newState not in frontSet) and (newState not in visited) and curLevel+1 <= depth:
                    tmpSol.append(action)
                    frontier.append([newState, curLevel+1, tmpSol])
    return False

def getState(node):
    return node[0]
def getDepth(node):
    return node[1]
def getSolution(node):
    return node[2]
def getMove(node):
    return node[3]

def depth_limited_search(problem, depth):
    frontier, visited, solution, level, move = [], [], [], 1, 0
    frontier.append([problem.getStartState(), level, solution, move])
    while frontier:
        node = frontier.pop()
        curState, curLevel, curSol, curMove = getState(node), getDepth(node), getSolution(node), getMove(node)
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
                frontSet.append(problem.getResult(getState(front), getMove(front)))
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
    frontier = Queue.PriorityQueue()
    solution = []
    move = 0
    node = (heuristic(problem.getStartState(), problem), problem.getStartState(), solution, move)
    frontier.put(node)
    visited = []
    while not frontier.empty():
        node = frontier.get()
        curState = node[1]
        curH = node[0]
        curSol = node[2]
        if problem.goalTest(curState):
            return curSol
        else:
            visited.append(curState)
            avaAction = problem.getActions(curState)
            for action in avaAction:
                tmpSol = list(curSol)
                newState = problem.getResult(curState, action)
                if newState not in visited:
                    tmpSol.append(action)
                    frontier.put((heuristic(newState, problem), newState, tmpSol))
    return False


# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
