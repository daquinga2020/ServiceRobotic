import math

def turn(dir, robott):
    
    if dir == "North":
        desire_dir = -math.pi/2
    elif dir == "East":
        # Puede ser pi o -pi
        if robott < 0:
            desire_dir = -math.pi
        else:
            desire_dir = math.pi
    elif dir == "South":
        desire_dir = math.pi/2
    elif dir == "West":
        desire_dir = 0.0
    
    error = round(robott - desire_dir, 4)
    
    return error
    '''while not alineated:
        
        if dir == "North":
            orientation = round(-math.pi/2, 3)
        elif dir == "East":
            orientation = round(-math.pi, 3)
        elif dir == "South":
            orientation = round(math.pi/2, 3)
        else:
            orientation = 0.0

        difference = orientation - current_dir
        
        if -err <= difference <= err:
            w = 0.5
            alineated = True'''


# 0 (oeste), -PI/2 (norte), PI (este) o PI/2 (sur). 
current_orientation = math.pi/2

# Hacia la derecha se gira con valor positivo
# Hacia la izquierda se gira con valor negativo
print(turn("East", current_orientation))