# -*- coding: utf-8 -*-
from datastructures import *

#----------------------------------------------------------------------

class Node:
  """
  This class is used to represent nodes of the search tree.  Each
  node contains a state representation, a reference to the node's
  parent node, a string that describes the action that generated
  the node's state from the parent state, the path cost g from
  the start node to this node, and the estimated path cost h
  from this node to the goal node.
  """
  def __init__(self, state, parent, action, g=0, h=0):
    self.state = state
    self.parent = parent
    self.action = action
    self.g = g
    self.h = h

  def __eq__(self, other):
    if other:
      return self.state == other.state
    else:
      return False
  
  def expand(self):
    successors = []
    for (newState, action) in self.state.next_states():
      newNode = Node(newState, self, action)
      successors.append(newNode)
    return successors
  
  def get_cost(self):
    return self.g + self.h

#------------------------------------------------------------
def uninformed_search(initial_state, goal_state, frontier):
  """
  Parametros:
  initial_state: estado inicial de busqueda (objeto de clase MissionariesState)
  goal_state: estado inicial de busqueda (objeto de clase MissionariesState)
  frontier: estructura de datos para contener los estados de la frontera (objeto de clase contenida en el modulo DataStructures)
  """
  """
  Rellenar con el codigo necesario para realizar una busqueda no informada siguiendo el pseudocodigo de los apuntes (Graph-Search)
  La funcion debe devolver una tupla con 3 variables:
    1. Nodo del grafo con el estado objetivo (None si no se ha alcanzado el objetivo)
    2. Numero de nodos expandidos (expanded)
    3. Numero de nodos generados (generated)
  """
  explored_nodes = Queue()
  expanded = 0
  generated = 0
  initial_node = Node(initial_state, None, None)

  frontier.insert(initial_node)
  
  while not frontier.is_empty():
    actual_node = frontier.remove()
    explored_nodes.insert(actual_node)
    
    if actual_node.state.__eq__(goal_state):
      return (actual_node, expanded, generated)
    
    expanded += 1
    for succesor in actual_node.expand():
      generated += 1
      succesor.g = actual_node.g + 1
      
      if succesor.state.__eq__(goal_state):
          return (succesor, expanded, generated)
      
      if not frontier.contains(succesor) and not explored_nodes.contains(succesor):
        frontier.insert(succesor)
      else:
        if type(frontier) == PriorityQueue:
          for i, node in enumerate(frontier.contents):
            if node and node.__eq__(succesor) and node.g > succesor.g:
                frontier.contents[i] = succesor
                break
  return (None, expanded, generated)
  
#----------------------------------------------------------------------
# Test functions for uninformed search

def breadth_first(initial_state, goal_state):
  frontier = Queue() # Indicar estructura de datos adecuada para breadth_first
  return uninformed_search(initial_state, goal_state, frontier)

def depth_first(initial_state, goal_state):
  frontier = Stack() # Indicar estructura de datos adecuada para depth_first
  return uninformed_search(initial_state, goal_state, frontier)

def uniform_cost(initial_state, goal_state):
  frontier = PriorityQueue(lambda node: node)
  return uninformed_search(initial_state, goal_state, frontier)
#----------------------------------------------------------------------

def informed_search(initial_state, goal_state, frontier, heuristic):
  """
  Parametros:
  initial_state: estado inicial de busqueda (objeto de clase MissionariesState)
  goal_state: estado inicial de busqueda (objeto de clase MissionariesState)
  frontier: estructura de datos para contener los estados de la frontera (objeto de clase contenida en el modulo DataStructures)
  heuristic: funcion heuristica utilizada para guiar el proceso de busqueda. Lafuncion recibe dos parametros (estado actual y estado objetivo) y devuelve una estimacion de coste entre ambos estados
  """
  
  """
  Rellenar con el codigo necesario para realizar una busqueda informada siguiendo el pseudocodigo de los apuntes (Graph-Search), modificada para actualizar el valor heuristico (h) de los nodos
  La funcion debe devolver una tupla con 3 variables:
    1. Nodo del grafo con el estado objetivo (None si no se ha alcanzado el objetivo)
    2. Numero de nodos expandidos (expanded)
    3. Numero de nodos generados (generated)
  """
  
  explored_nodes = Queue()
  expanded = 0
  generated = 0
  initial_node = Node(initial_state, None, None)
  
  frontier.insert(initial_node)
  
  while not frontier.is_empty():
    actual_node = frontier.remove()
    explored_nodes.insert(actual_node)
    
    if actual_node.state.__eq__(goal_state):
      return actual_node
    
    expanded += 1
    for succesor in actual_node.expand():
      generated += 1
      succesor.g = actual_node.g + 1
      succesor.h = heuristic(succesor.state, goal_state)
      
      if succesor.state.__eq__(goal_state):
        return (succesor, expanded, generated)
      
      
      if not frontier.contains(succesor) and not explored_nodes.contains(succesor):
        frontier.insert(succesor)
  return (None, expanded, generated)
  
#----------------------------------------------------------------------
# Test functions for informed search

def greedy(initial_state, goal_state, heuristic):
  frontier = PriorityQueue(lambda x : x) # Indicar estructura de datos adecuada para greedy
  return informed_search(initial_state, goal_state, frontier, heuristic)

def a_star(initial_state, goal_state, heuristic):
  frontier = PriorityQueue(lambda x : x) # Indicar estructura de datos adecuada para A*
  return informed_search(initial_state, goal_state, frontier, heuristic) 

#---------------------------------------------------------------------
# Heuristic functions

# Heuristica basada en el numero de viajes que faltan para llevar a todos los personajes al otro lado del rio
def h1(current_state, goal_state):
  remaining = abs(goal_state.miss[1] - current_state.miss[1] + goal_state.cann[1] - current_state.cann[1])
  return remaining / current_state.capacity

# Heuristica que penaliza la desigualdad de canibales respecto a misioneros en los lados del rio
def h2(current_state, goal_state):
  remaining = abs(goal_state.miss[1] - current_state.miss[1] + goal_state.cann[1] - current_state.cann[1])
  # Penalización por desequilibrio (si hay más caníbales que misioneros en cualquier orilla)
  penalty = 0
  if current_state.miss[0] > 0 and current_state.cann[0] > current_state.miss[0]:
    penalty += 1
  if current_state.miss[1] > 0 and current_state.cann[1] > current_state.miss[1]:
    penalty += 1
  return (remaining / current_state.capacity) + penalty

#----------------------------------------------------------------------
def show_solution(node, expanded, generated):
  path = []
  while node != None:
    path.insert(0, node)
    node = node.parent
  if path:
    print ("Solution took %d steps" % (len(path) - 1))
    print (path[0].state)
    for n in path[1:]:
      print ('%s %s %s' % (n.action[0], n.action[1], n.action[2]))
      print (n.state)
  print ("Nodes expanded:  %d" % expanded)
  print ("Nodes generated: %d\n" % generated)
