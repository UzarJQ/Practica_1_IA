
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
        
        #Verificar que la cantidad de personajes en el bote no superen la capacidad del mismo
        if self.capacity >= (movement[1]+movement[2]):
            
            #Verificar que en el bote no hayan mas canibales que missioneros
            if movement[2] > movement[1]:
                return None
            
            #Verificar que el bote solo pueda moverse a la derecha si el estado actual es del lado izquierdo
            if self.boat_position == "left" and movement[0] == ">":
                #Verifica que al quitar X misioneros del lado izquierdo y sumarlos al lado derecho, en ambos lados del rio haya mas misioneros que canibales
                if(self.miss[0] - movement[1] ) >= self.cann[0] and (self.miss[1] + movement[1]) >= self.cann[1]:
                    self.miss[0] -= movement[1]
                    self.miss[1] += movement[1]
                else:
                    return None 
                #Verificar que al quitar Y canibales del lado izquierdo y sumarlos al derecho, en ambos lados del rio haya mas misioneros que canibales
                if (self.cann[0] - movement[2]) <= self.miss[0] and (self.cann[1] + movement[2]) <= self.miss[1]:
                    self.cann[0] -= movement[2]
                    self.cann[1] += movement[2]
                else:
                    return None
            #Verificar que el bote solo pueda moverse a la izquierda si el estado actual es del lado derecho
            elif self.boat_position == "right" and movement[0] == "<":
                #Verifica que al quitar X misioneros del lado derecho y sumarlos al lado izquierdo, en ambos lados del rio haya mas misioneros que canibales
                if(self.miss[1] - movement[1]) >= self.cann[1] and (self.miss[0] + movement[1]) >= self.cann[0]:
                    self.miss[1] -= movement[1]
                    self.miss[0] += movement[1]
                else:
                    return None
                #Verificar que al quitar Y canibales del lado derecho y sumarlos al izquierdo, en ambos lados del rio haya mas misioneros que canibales
                if (self.cann[1] - movement[2]) <= self.miss[1] and (self.cann[0] + movement[2]) <= self.miss[0]:
                    self.cann[1] -= movement[2]
                    self.cann[0] += movement[2]
                else:
                    return None
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
                        action = f">{misionary}{cannibal}"  # Mover hacia la derecha
                    else:
                        action = f"<{misionary}{cannibal}"  # Mover hacia la izquierda
                    possible_actions.append(action)
    
    # Probar cada acción y verificar si genera un estado válido
        for action in possible_actions:
            new_state = MissionariesState(self.miss[0], self.cann[0], self.miss[1], self.cann[1], self.boat_position, self.capacity)
            result = new_state.succ(action)
        
            if result is not None:  # Si el estado es válido
                if self.boat_position == "left": 
                    new_state.boat_position = "right" 
                else:
                    new_state.boat_position = "left"
                
                new_states.append((new_state, action))
        
        return new_states