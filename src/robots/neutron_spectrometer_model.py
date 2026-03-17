import math
import random


class NeutronSpectrometerModel():
    
    def __init__(self):
        self._t = 0
        self._neutron_gen_values = {
            "min":0,
            "max":5000,
            "period": 20,
            "offset": 0,
        }
        self.is_near_water:bool = False
        self._std_neutron = 10
    
    def get_next_count(self):
        if self.is_near_water:
            count =  max(0, 50 + random.gauss(0.0, self._std_neutron))
        else:
            count = max(0, 200 + random.gauss(0.0, self._std_neutron))

        return int(count)
    
    def _generate_sine_value(self, t, min_val, max_val, period, offset):
        """Generate a sine wave value at time t with given parameters."""
        omega = 2 * math.pi / period
        amplitude = (max_val - min_val) / 2
        center = (max_val + min_val) / 2
        return center + amplitude * math.sin(omega * (t + offset))