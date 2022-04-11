import carla
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)


def draw_waypoints(waypoints, road_id=None, life_time=50.0):
    for waypoint in waypoints:

        if (waypoint.road_id == road_id):
            self.world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                         color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                         persistent_lines=True)


waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)
draw_waypoints(waypoints, road_id=10, life_time=20)