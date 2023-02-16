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
from util import PriorityQueue, Stack, Queue

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

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    """
    Esta es la implementación de un DFS iterativo, se hace uso de una stack para contener los estados,
    una lista de visitados para marcar donde ya el pacman hubiera pasado, se empieza con el nodo inicial
    y se le hace push a la stack. Mientras la pila este con elementos y no llegue al estado objetivo 
    que es cuando come mira los posibles movimientos del nodo en el  que se encuentra, luego de eso marca 
    como visitados para no crear un loop y mete el siguiente estado a la pila. 
    Esta recorre en profundidad como su nombre lo indica 
    """
    stack = Stack()
    visited = list()
    start = (problem.getStartState(), [])
    stack.push(start)
    while stack.isEmpty() == False:
        currentPos, path = stack.pop()
        if problem.isGoalState(currentPos):
            return path
        else:
            visited.append(currentPos)
            for node, direction, cost in problem.getSuccessors(currentPos):
                if node not in visited:
                    nextWay = (node, path + [direction])
                    stack.push(nextWay)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    """
    Esta es la implementación de un BFS, se hace uso de una queue para contener los estados,
    una lista de visitados para marcar donde ya el pacman hubiera pasado, se empieza con el nodo inicial
    y se le hace push a la queue. Mientras la cola este con elementos y no llegue al estado objetivo 
    que es cuando come mira los posibles movimientos del nodo en el que se encuentra, luego de eso marca 
    como visitados para no crear un loop y mete el siguiente estado a la queue.
    """
    queue = Queue()
    visited = list()
    start = (problem.getStartState(), [])
    queue.push(start)
    visited.append(problem.getStartState())
    while queue.isEmpty() == False:
        currentPos, path = queue.pop()
        if problem.isGoalState(currentPos):
            return path
        else:
            for node, direction, cost in problem.getSuccessors(currentPos):
                nextWay = (node, path + [direction])
                if node not in visited:
                    queue.push(nextWay)
                    visited.append(node)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    """
    Esta es la implementación de ucs, tiene gran parecido con dijsktra, para este se necesita un heap
    ya que se tendrá en cuenta su costo, de igual forma que los otros algoritmos se hace uso de visitados 
    para evitar loops, luego se hace push al heap y procede mientras tenga estados y no encuentre su estado
    objetivo, siempre se agrega un nuevo estado con el nodo en el que se encuentra, su costo y el camino al
    heap.
    """
    heap = PriorityQueue()
    visited = list()
    start = (problem.getStartState(), 0, [])
    heap.push(start, 0)
    while heap.isEmpty() == False:
        currentPos, costA, path = heap.pop()
        if problem.isGoalState(currentPos):
            return path
        if currentPos not in visited:
            visited.append(currentPos)
            for node, direction, cost in problem.getSuccessors(currentPos):
                nextWay = (node, costA + cost, path + [direction])
                heap.push(nextWay, costA + cost)



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    """
    Es casi parecido al ucs con el unico cambio que es cuando mete el nodo al heap
    tiene encuenta un heuristico definido.
    """
    heap = PriorityQueue()
    visited = list()
    start = (problem.getStartState(), 0, [])
    heap.push(start, heuristic(problem.getStartState(), problem))
    while heap.isEmpty() == False:
        currentPos, costA, path = heap.pop()
        if problem.isGoalState(currentPos):
            return path
        if currentPos not in visited:
            visited.append(currentPos)
            for node, direction, cost in problem.getSuccessors(currentPos):
                nextWay = (node, costA + cost, path + [direction])
                heap.push(nextWay, costA + cost + heuristic(node, problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
