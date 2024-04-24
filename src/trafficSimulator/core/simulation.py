from .vehicle_generator import VehicleGenerator
from .geometry.quadratic_curve import QuadraticCurve
from .geometry.cubic_curve import CubicCurve
from .geometry.segment import Segment
from .vehicle import Vehicle


COLLISION_DISTANCE = 2


class Simulation:
    def __init__(self):
        self.segments = []
        self.vehicles = {}
        self.vehicle_generator = []

        self.t = 0.0
        self.frame_count = 0
        self.dt = 1/60  
        # self.dt = 1


    def add_vehicle(self, veh):
        self.vehicles[veh.id] = veh
        if len(veh.path) > 0:
            self.segments[veh.path[0]].add_vehicle(veh)

    def add_segment(self, seg):
        self.segments.append(seg)

    def add_vehicle_generator(self, gen):
        self.vehicle_generator.append(gen)

    
    def create_vehicle(self, **kwargs):
        veh = Vehicle(kwargs)
        self.add_vehicle(veh)

    def create_segment(self, *args):
        seg = Segment(args)
        self.add_segment(seg)

    def create_quadratic_bezier_curve(self, start, control, end):
        cur = QuadraticCurve(start, control, end)
        self.add_segment(cur)

    def create_cubic_bezier_curve(self, start, control_1, control_2, end):
        cur = CubicCurve(start, control_1, control_2, end)
        self.add_segment(cur)

    def create_vehicle_generator(self, **kwargs):
        gen = VehicleGenerator(kwargs)
        self.add_vehicle_generator(gen)


    def run(self, steps):
        for _ in range(steps):
            self.update()

    def update(self):
        # Update vehicles
        for segment in self.segments:
            if len(segment.vehicles) != 0:
                self.vehicles[segment.vehicles[0]].v_max = segment.get_length()
                self.vehicles[segment.vehicles[0]].update(None, self.dt)
            for i in range(1, len(segment.vehicles)):
                self.vehicles[segment.vehicles[i]].v_max = segment.get_length()
                self.vehicles[segment.vehicles[i]].update(self.vehicles[segment.vehicles[i-1]], self.dt)
                if self.vehicles[segment.vehicles[i]].x >= segment.get_length():
                    self.vehicles[segment.vehicles[i]].x = segment.get_length()

        # Check roads for out of bounds vehicle
        for segment in self.segments:
            # If road has no vehicles, continue
            if len(segment.vehicles) == 0: continue
            # If not
            vehicle_id = segment.vehicles[0]
            vehicle = self.vehicles[vehicle_id]
            # If first vehicle is out of road bounds
            if vehicle.x >= segment.get_length():
                same = False
                # If vehicle has a next road
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    vehicle.v = vehicle.v_max
                    # Update current road to next road
                    vehicle.current_road_index += 1
                    # Add it to the next road
                    next_road_index = vehicle.path[vehicle.current_road_index]
                    if next_road_index == -1 or next_road_index == vehicle.path[vehicle.current_road_index - 1]:
                        same = True
                        vehicle.v = 0
                        vehicle.x = segment.get_length()
                        if vehicle.stop_timesteps < vehicle.max_stop_timesteps:
                            vehicle.stop_timesteps += 1
                            vehicle.current_road_index -= 1
                        else:
                            vehicle.stop_timesteps = 0
                    else:
                        self.segments[next_road_index].vehicles.append(vehicle_id)
                if not same:
                    # Reset vehicle properties
                    vehicle.x = 0
                    # In all cases, remove it from its road
                    segment.vehicles.popleft() 

        # Update vehicle generators
        for gen in self.vehicle_generator:
            gen.update(self)
        # Increment time
        self.t += self.dt
        self.frame_count += 1

        # Check if any vehicles collided
        # for segment in self.segments:
        #     if len(segment.vehicles) != 0:
        #         for i in range(1, len(segment.vehicles)):
        #             if abs(self.vehicles[segment.vehicles[i]].x - self.vehicles[segment.vehicles[i-1]].x) < COLLISION_DISTANCE:
        #                 self.vehicles[segment.vehicles[i]].collided = True
        #                 self.vehicles[segment.vehicles[i-1]].collided = True
        #                 print("Collision detected at time:", self.t)
        #                 print("Vehicle 1:", self.vehicles[segment.vehicles[i]].x)
        #                 print("Vehicle 2:", self.vehicles[segment.vehicles[i-1]].x)
        # Overlap at intersection is also considered as collision


        # Get all vehicles and their positions
        vehicles = []
        for segment in self.segments:
            for vehicle_id in segment.vehicles:
                vehicle = self.vehicles[vehicle_id]
                # print(self.t)
                t = vehicle.x / segment.get_length()
                vp = (vehicle, segment.compute_x(t), segment.compute_y(t))
                vehicles.append(vp)
                print(vp)
        
            
        # Check if any vehicles collided by comparing their positions
        for i in range(len(vehicles)):
            for j in range(i+1, len(vehicles)):
                if (vehicles[i][1] - vehicles[j][1])**2 + (vehicles[i][2] - vehicles[j][2])**2 < COLLISION_DISTANCE**2:
                    vehicles[i][0].collided = True
                    vehicles[j][0].collided = True
                    print("Collision detected at time:", self.t)
                    print("Vehicle 1:", vehicles[i])
                    print("Vehicle 2:", vehicles[j])


