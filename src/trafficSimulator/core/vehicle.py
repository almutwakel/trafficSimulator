import uuid
import numpy as np

class Vehicle:
    def __init__(self, config={}):
        # Set default configuration
        self.set_default_config()


        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

        self.max_stop_timesteps = 50
        self.stop_timesteps = 0
        self.collided = False
        
    def set_default_config(self):    
        self.id = uuid.uuid4()

        self.l = 4
        self.s0 = 4
        self.T = 1
        self.v_max = 10
        self.a_max = 0
        self.b_max = 0

        self.path = []
        self.current_road_index = 0

        self.x = 0
        self.v = 0
        self.a = 0
        self.stopped = False
        self.color = (0, 0, 255)

    def init_properties(self):
        self.sqrt_ab = 2*np.sqrt(self.a_max*self.b_max)
        self._v_max = self.v_max

    def update(self, lead, dt):
        # Update position and velocity

        self.x += self.v*dt

        if self.stopped:
            self.v = 0
        else:
            self.v = self.v_max
        