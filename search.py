# -*- coding: utf-8 -*-
from datastructures import *
import math

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
  initial_node = Node(initial_state, None, None)
  expanded = 0
  generated = 0
  explored_nodes = Queue()
  
  frontier.insert(initial_node)
  
  while frontier.contents:
    leaf_node: Node = frontier.remove()
    
    if leaf_node:
      if leaf_node.state.__eq__(goal_state):
        return (leaf_node, expanded, generated)
      
    explored_nodes.insert(leaf_node)
    
    expanded += 1
    for succesor in leaf_node.expand():
      generated += 1
      founded = False
      
      if explored_nodes.is_empty() and frontier.is_empty():
        frontier.insert(succesor)
      else:
        for explored in explored_nodes.contents:
          if(succesor.state.__eq__(explored.state)):
            founded = True
            break
        
        if not founded:
          for leaf in frontier.contents:
            if(succesor.state.__eq__(leaf.state)):
              founded = True
              break
          
        if not founded:
          frontier.insert(succesor)
  return (None, expanded, generated)
  
#----------------------------------------------------------------------
# Test functions for uninformed search

def breadth_first(initial_state, goal_state):
  frontier = Queue() # Indicar estructura de datos adecuada para breadth_first
  return uninformed_search(initial_state, goal_state, frontier)

def depth_first(initial_state, goal_state):
  frontier = Queue() # Indicar estructura de datos adecuada para depth_first
  return uninformed_search(initial_state, goal_state, frontier)

def uniform_cost(initial_state, goal_state):
  frontier = PriorityQueue(lambda node: node.g) # Use the path cost g for priority
  initial_node = Node(initial_state, None, None)
  
  expanded = 0
  generated = 0
  
  frontier.insert(initial_node)
  
  explored_nodes = Queue()
  while not frontier.is_empty():
    leaf_node: Node = frontier.remove()
    
    if leaf_node.state.__eq__(goal_state):
      return (leaf_node, expanded, generated)
    
    expanded += 1
    for succesor in leaf_node.expand():
      generated += 1
      succesor.g = leaf_node.g + 1
      
      if succesor.state.__eq__(goal_state):
        return (succesor, expanded, generated)
      
      if not explored_nodes.contains(succesor) and not frontier.contains(succesor):
        explored_nodes.insert(leaf_node)
        frontier.insert(succesor)
      else:
        for i, node in enumerate(frontier.contents):
            if node and node.__eq__(succesor) and node.g > succesor.g:
                frontier.contents[i] = succesor
                break
  return (None, expanded, generated)
#----------------------------------------------------------------------

def informed_search(initial_state, goal_state, frontier, heuristic):
  """
  Parametros:
  initial_state: estado inicial de busqueda (objeto de clase MissionariesState)
  goal_state: estado inicial de busqueda (objeto de clase MissionariesState)
  frontier: estructura de datos para contener los estados de la frontera (objeto de clase
    contenida en el modulo DataStructures)
  heuristic: funcion heuristica utilizada para guiar el proceso de busqueda. La
    funcion recibe dos parametros (estado actual y estado objetivo) y devuelve
    una estimacion de coste entre ambos estados
  """

  initial_node = Node(initial_state, None, None)
  frontier.insert(initial_node)
  expanded = 0
  generated = 0
  
  while frontier:
    current_node = frontier.remove()
    
    if current_node.__eq__(Node(goal_state)):
      return current_node
    
    expanded += 1
    
    for child_node in current_node.expand():
      if not frontier.contains(child_node.state): 
        generated += 1
        child_node.g = current_node.g + 1
        child_node.h = heuristic(child_node.state, goal_state)
        frontier.insert(child_node)
  
  """
  Rellenar con el codigo necesario para realizar una busqueda informada
  siguiendo el pseudocodigo de los apuntes (Graph-Search), modificada para
  actualizar el valor heuristico (h) de los nodos
  La funcion debe devolver una tupla con 3 variables:
    1. Nodo del grafo con el estado objetivo (None si no se ha alcanzado el objetivo)
    2. Numero de nodos expandidos (expanded)
    3. Numero de nodos generados (generated)
  """
  
  return (None, expanded, generated)
  
#----------------------------------------------------------------------
# Test functions for informed search

def greedy(initial_state, goal_state, heuristic):
  frontier = PriorityQueue(lambda x : x) # Indicar estructura de datos adecuada para greedy
  return informed_search(initial_state, goal_state, frontier, heuristic)

def a_star(initial_state, goal_state, heuristic):
  frontier = PriorityQueue # Indicar estructura de datos adecuada para A*
  return informed_search(initial_state, goal_state, frontier, heuristic) 

#---------------------------------------------------------------------
# Heuristic functions

# Heuristica basada en el numero de viajes necesarios para llevar a todos los personajes al otro lado del rio
def h1(current_state, goal_state):
  remaining = current_state.miss[0] + current_state.cann[0]
  return (remaining + 1) // current_state.capacity

# Heuristica basada en que al haber mas misioneros del lado derecho, se necesitaran menos movimientos
def h2(current_state, goal_state):
  return goal_state.miss[1] - current_state.miss[1]

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
