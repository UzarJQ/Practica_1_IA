# -*- coding: utf-8 -*-
class MissionariesState:
  """
  This class is used to represent a state of the missionaries
  and cannibals problem. Each state contains the number of
  missionaries and cannibals in each shore, the position
  of the boat, and the capacity of the boat to determine
  whether a state is valid or not.
  """
  
  def __init__(self, left_miss, left_cann, right_miss, right_cann, boat_position, capacity=2):
    self.miss = (left_miss, right_miss)  # missionaries in left and right shores
    self.cann = (left_cann, right_cann)  # cannibals in left and right shores
    self.boat_position = boat_position   # boat position ('left', 'right')
    self.capacity = capacity             # boat capacity (missionaries+cannibals)
    
  def __str__(self):
    to_str = "(%d, %d)" % (self.miss[0], self.cann[0])
    if self.boat_position == "left":
      to_str += " (||)      "
    else:
      to_str += "      (||) "
    to_str += "(%d, %d)" % (self.miss[1], self.cann[1])
    return to_str
    
  def __eq__(self, other):
    return self.miss == other.miss and self.cann == other.cann and self.boat_position == other.boat_position

  def succ(self, action):
    """
    Rellenar con el codigo necesario para generar un nuevo estado a partir del actual
    y una accion proporcionada como parametro. La accion tiene el formato '<MC' o '>MC',
    donde M es el numero de misioneros y C el numero de canibales que pasan al otro lado
    del rio. La funcion debe devolver None si el estado generado es invalido segun
    las especificaciones del problema
    """
    movement = list(action)
    movement[1] = int(movement[1])
    movement[2] = int(movement[2])
    
    new_miss = [0,0]
    new_cann = [0,0]
    new_direction = ""
    #Verificar que la cantidad de personajes en el bote no superen su capacidad
    if self.capacity >= (movement[1] + movement[2]):
      #Verificar que en el bote no hayan mas canibales que missioneros
      if movement[2] > movement[1] and movement[1] > 0:
        return None
      #Determinar el nuevo estado a partir de la dirección del movimiento
      if movement[0] == ">":
        new_miss[0] = self.miss[0] - movement[1]
        new_miss[1] = self.miss[1] + movement[1]
        new_cann[0] = self.cann[0] - movement[2]  
        new_cann[1] = self.cann[1] + movement[2]
        new_direction = "right"
      elif movement[0] == "<":
        new_miss[0] = self.miss[0] + movement[1]
        new_miss[1] = self.miss[1] - movement[1]
        new_cann[0] = self.cann[0] + movement[2]
        new_cann[1] = self.cann[1] - movement[2]
        new_direction = "left"
      else:
        return None
      #Verificar que en ambos lados del rio no hayan mas canibales que missioneros (excepto si no hay missioneros)
      valid_movement = (
        (new_miss[0] >= new_cann[0] or new_miss[0] == 0) and 
        (new_miss[1] >= new_cann[1] or new_miss[1] == 0) and 
        new_miss[0] >= 0 and 
        new_cann[0] >= 0 and 
        new_miss[1] >= 0 and 
        new_cann[1] >= 0
      )
      if(valid_movement):
        return MissionariesState(new_miss[0], new_cann[0], new_miss[1], new_cann[1], new_direction, self.capacity)
      else:
        return None
    else:
      return None

  def next_states(self):
    new_states = []
    possible_actions = []
    """
    Rellenar con el codigo necesario para generar la lista de nuevos estados accesibles
    desde el estado actual, aplicando las diferentes acciones posibles. Los estados deben 
    ser validos segun las especificaciones del problema. La lista debe estar formada por 
    pares (nuevo_estado, accion)
    """
    for misionary in range(self.capacity + 1):
      for cannibal in range(self.capacity + 1 - misionary):
        if misionary + cannibal > 0:  # Al menos debe haber una persona en el bote
          if self.boat_position == "left":
            action = ">{}{}".format(misionary, cannibal)
          else:
            action = "<{}{}".format(misionary, cannibal)
          possible_actions.append(action)
    # Probar cada acción y verificar si genera un estado válido
    for action in possible_actions:
      actual_state = MissionariesState(self.miss[0], self.cann[0], self.miss[1], self.cann[1], self.boat_position, self.capacity)
      succ_state = actual_state.succ(action)
      if succ_state is not None:  # Si el estado es válido
        new_states.append((succ_state, action))
    return new_states