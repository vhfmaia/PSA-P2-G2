
import sys
import time
from datetime import datetime
from matplotlib import cm
import open3d as o3d
import carla
import numpy as np


# Get colormap
VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])


# Get point colors
def lidar_callback(point_cloud, point_list):
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    points = data[:, :-1]

    points[:, :1] = -points[:, :1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)


def generate_lidar_bp(blueprint_library, delta):

    # Generate lidar blueprint
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

    # Define sensor parameters
    lidar_bp.set_attribute('noise_stddev', '0.2')
    lidar_bp.set_attribute('upper_fov', str(15.0))
    lidar_bp.set_attribute('lower_fov', str(-25.0))
    lidar_bp.set_attribute('channels', str(64.0))
    lidar_bp.set_attribute('range', str(100.0))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(500000))

    return lidar_bp


def main():

    # Connect to carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(40.0)


    # Load world
    world = client.load_world("Town07")
    world.set_weather(carla.WeatherParameters.ClearNoon)


    try:
        # Traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)


        # Update world settings
        settings = world.get_settings()

        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        world.apply_settings(settings)


        # Select vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]


        # Start position
        transform = carla.Transform()

        transform.location.x = 127.0
        transform.location.y = 237.4
        transform.location.z = 1.5

        transform.rotation.yaw = 180
        transform.rotation.pitch = 0
        transform.rotation.roll = 0


        # Spawn vehicle at start position
        vehicle = world.spawn_actor(vehicle_bp, transform)


        # Move vehicle
        vehicle.set_autopilot(True)
        # control = carla.VehicleControl()
        # control.throttle = 0.2
        # vehicle.apply_control(control)


        # Spawn lidar
        lidar_bp = generate_lidar_bp(blueprint_library, delta)
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        point_list = o3d.geometry.PointCloud()


        # Update
        lidar.listen(lambda data: lidar_callback(data, point_list))


        # Open window
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Carla Lidar',
            width=960,
            height=540,
            left=480,
            top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True


        # Update window
        frame = 0
        dt0 = datetime.now()
        while True:
            if frame == 2:
                vis.add_geometry(point_list)
            vis.update_geometry(point_list)
            vis.poll_events()
            vis.update_renderer()

            # Fix Open3D jittering issues:
            time.sleep(0.005)
            world.tick()


            # Display FPS in terminal
            process_time = datetime.now() - dt0
            sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
            sys.stdout.flush()
            dt0 = datetime.now()
            frame += 1

    # Stop
    finally:
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle.destroy()
        lidar.destroy()
        vis.destroy_window()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
