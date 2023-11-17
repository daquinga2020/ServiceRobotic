import math

def clean_data_laser(data_laser):
    
    values = data_laser.values # tuple
    clean_data =  []
    
    for i in range(181):
        if not math.isnan(values[i]) and not math.isinf(values[i]):
            clean_data.append((values[i], i))
    
    return clean_data # [(value, angle), ...]