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

        # Check if any vehicles collided using the compute_x(t) and compute_y(t) functions of the segments
        for s1 in range(len(self.segments)):
            segment = self.segments[s1]
            if len(segment.vehicles) != 0:
                for i in range(len(segment.vehicles)):
                    veh1 = self.vehicles[segment.vehicles[i]]
                    for s2 in range(len(self.segments[s1+1:])):
                        segment2 = self.segments[s1+1:][s2]
                        if len(segment2.vehicles) == 0:
                            continue
                        for j in range(len(segment2.vehicles)):
                            veh2 = self.vehicles[segment2.vehicles[j]]
                            x1 = segment.compute_x(self.t % 1.0)
                            y1 = segment.compute_y(self.t % 1.0)
                            x2 = segment2.compute_x(self.t % 1.0)
                            y2 = segment2.compute_y(self.t % 1.0)
                            print("Vehicle 1:", x1, y1)
                            print("Vehicle 2:", x2, y2)
                            if abs(x1 - x2) < COLLISION_DISTANCE and abs(y1 - y2) < COLLISION_DISTANCE:
                                veh1.collided = True
                                veh2.collided = True
                                print("Collision detected at time:", self.t)


