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
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action
        self.g = 0
        self.h = 0

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

#----------------------------------------------------------------------

def graph_search(initial_node, goal_state):
    explored_nodes = set()
    frontier = Queue()
    frontier.insert(initial_node)

    while frontier:
        current_node = frontier.remove()

        if current_node.__eq__(Node(goal_state)):
            return current_node

        explored_nodes.add(current_node.state)

        for child_node in current_node.expand():
            if child_node.state not in [n.state for n in frontier] and child_node.state not in explored_nodes:
                frontier.append(child_node)

    return None

    
#------------------------------------------------------------
def uninformed_search(initial_state, goal_state, frontier):

    """
    Parametros:
    initial_state: estado inicial de busqueda (objeto de clase MissionariesState)
    goal_state: estado inicial de busqueda (objeto de clase MissionariesState)
    frontier: estructura de datos para contener los estados de la frontera (objeto de clase
    contenida en el modulo DataStructures)
    """
    explored_nodes = set()  # Almacena los nodos ya explorados
    frontier = Queue()  # Usa la clase Queue como cola FIFO
    frontier.insert(initial_node)  # Inserta el nodo inicial

    initial_node = Node(initial_state, None, None)
    graph_search(initial_node)
    expanded = 0
    generated = 0
    
    while not frontier.is_empty():  # Mientras la frontera no esté vacía
        current_node = frontier.remove()  # Extrae el primer nodo de la cola
        expanded += 1  # Expande el nodo actual

        if current_node.state == goal_state:  # Si encontramos el estado objetivo
            return (current_node, expanded, generated)  # Retorna el nodo, nodos expandidos y generados

        explored_nodes.add(current_node.state)  # Marca el nodo como explorado

        # Expande los nodos hijos y los añade a la frontera
        for child_node in current_node.expand():
            if child_node.state not in explored_nodes and not frontier.contains(child_node):
                frontier.insert(child_node)  # Añade a la frontera los nodos no explorados
                generated += 1  # Incrementa el contador de nodos generados

    """ return None  # Si no se encuentra el objetivo, retorna None"""
    """
    Rellenar con el codigo necesario para realizar una busqueda no informada
    siguiendo el pseudocodigo de los apuntes (Graph-Search)
    La funcion debe devolver una tupla con 3 variables:
        1. Nodo del grafo con el estado objetivo (None si no se ha alcanzado el objetivo)
        2. Numero de nodos expandidos (expanded)
        3. Numero de nodos generados (generated)
    """
    
    return (None, expanded, generated)
    
#----------------------------------------------------------------------
# Test functions for uninformed search

def breadth_first(initial_state, goal_state):
    frontier = Queue(); # Indicar estructura de datos adecuada para breadth_first
    return uninformed_search(initial_state, goal_state, frontier)

def depth_first(initial_state, goal_state):
    frontier = None # Indicar estructura de datos adecuada para depth_first
    return uninformed_search(initial_state, goal_state, frontier)

def uniform_cost(initial_state, goal_state):
    frontier = None # Indicar estructura de datos adecuada para uniform_cost
    return uninformed_search(initial_state, goal_state, frontier)


#----------------------------------------------------------------------

def informed_search(initial_state, goal_state, frontier: Queue, heuristic):

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
        current_node: Node = frontier.remove()
        
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
    frontier = None # Indicar estructura de datos adecuada para greedy
    return informed_search(initial_state, goal_state, frontier, heuristic)

def a_star(initial_state, goal_state, heuristic):
    frontier = None # Indicar estructura de datos adecuada para A*
    return informed_search(initial_state, goal_state, frontier, heuristic) 

#---------------------------------------------------------------------
# Heuristic functions

def h1(current_state, goal_state):
    return 0


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
            print ('%s %d %d' % (n.action[0], n.action[1], n.action[2]))
            print (n.state)
    print ("Nodes expanded:  %d" % expanded)
    print ("Nodes generated: %d\n" % generated)

