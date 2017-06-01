# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).

  You do not need to change anything in this class, ever.
  """

  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        child_list = []
		for child in problem.getSuccessors(self.state)[0]]:
			child_list.apend(child)
		return child_list

    def child_node(self, problem):
        """[Figure 3.10]"""
        next = problem.getSuccessors(self.state)
        return Node(next[0], self.state, parent=node, action=next[1], path_cost=next[2])
		
		
		
    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

		
		
		
def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  node = problem.getStartState()
  explored = set()
  if problem.isGoalState(node):
      return node
  frontier = Stack()
  frontier.append(node)
  explored = set()
  while frontier:
      node = frontier.pop()
      explored.add(node.state)
      for child in node.expand(problem):
          if child.state not in explored and child not in frontier:
              if problem.goal_test(child.state):
                  return child
              frontier.append(child)
  return None


def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  node = problem.getStartState()
  if problem.isGoalState(node):
      return node
  frontier = Queue()
  frontier.append(node)
  explored = set()
  while frontier:
      node = frontier.pop()
      explored.add(node.state)
      for child in node.expand(problem):
          if child.state not in explored and child not in frontier:
              if problem.goal_test(child.state):
                  return child
              frontier.append(child)
  return None



def path_cost_function(node):
	return node.path_cost
  
  
def total_cost_function(node, goal):
	return node.path_cost + util.manhattanDistance(node, goalstate)
  
  
def uniformCostSearch(problem):
	"Search the node of least total cost first. "
	node = porblem.getStartState()
	if problem.isGoalState(node.state):
		return node
	frontier = PriorityQueueWithFunction()
	explored = set()
    
    while frontier:
        node = frontier.pop()	 
		if problem.isGoalState(node.state):
			return node
		explored.add(node.state)	
		for child in node.expand(probelm):
			if child.state not in explored and child not in frontier:
				frontier.append(child)
		    elif child in frontier:
				incumbent = frontier[child]
                if path_cost_function(child) < path_cost_function(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None
				
	

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

  
def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  node = porblem.getStartState()
	if problem.isGoalState(node.state):
		return node
	frontier = PriorityQueueWithFunction()
	explored = set()
    
    while frontier:
        node = frontier.pop()	 
		if problem.isGoalState(node.state):
			return node
		explored.add(node.state)	
		for child in node.expand(probelm):
			if child.state not in explored and child not in frontier:
				frontier.append(child)
		    elif child in frontier:
				incumbent = frontier[child]
                if total_cost_function(child) < total_cost_function(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None
	
    
  
# Abbreviations

problem = SearchProblem()
bfs = breadthFirstSearch(problem)
#dfs = depthFirstSearch(problem)
print bfs
#astar = aStarSearch(problem)
#ucs = uniformCostSearch(problem)
