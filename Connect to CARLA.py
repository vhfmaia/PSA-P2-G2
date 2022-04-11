##Connect to carla Client
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)

##Create Waypoint
def draw_waypoints(waypoints, road_id=None, life_time=50.0):
    for waypoint in waypoints:

        if (waypoint.road_id == road_id):
            self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                         color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                         persistent_lines=True)


waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)
draw_waypoints(waypoints, road_id=10, life_time=20)

##Import blueprint
vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0]

##Spawn Imported car into waypoint
filtered_waypoints = []
for waypoint in waypoints:
    if(waypoint.road_id == 10):
      filtered_waypoints.append(waypoint)

spawn_point = filtered_waypoints[42].transform
spawn_point.location.z += 2
vehicle = client.get_world().spawn_actor(vehicle_blueprint, spawn_point)

##initialize the controller
from agents.navigation.controller import VehiclePIDController
custom_controller = VehiclePIDController(vehicle, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

##Track Waypoint
target_waypoint = filtered_waypoints[50]
client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False,
                           color=carla.Color(r=255, g=0, b=0), life_time=20,
                           persistent_lines=True)
ticks_to_track = 20
for i in range(ticks_to_track):
	control_signal = custom_controller.run_step(1, target_waypoint)
	vehicle.apply_control(control_signal)
