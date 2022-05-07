
# Libraries
import carla
import time
import pygame
import numpy as np


# Connect to carla
client = carla.Client('localhost', 2000)
client.set_timeout(60.0)


# Change map
world = client.load_world("Town02")
world.set_weather(carla.WeatherParameters.ClearNoon)


# Select vehicle
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]


# Start position
transform = carla.Transform()

transform.location.x = 127
transform.location.y = 237.4
transform.location.z = 1.85

transform.rotation.yaw = 180
transform.rotation.pitch = 0
transform.rotation.roll = 0


# Get spectator
vehicle = world.spawn_actor(vehicle_bp, transform)

spectator = world.get_spectator()
sp_transform = carla.Transform(transform.location + carla.Location(z=30, x=-25),
    carla.Rotation(yaw=90, pitch=-90))
spectator.set_transform(sp_transform)


# Move vehicle
control = carla.VehicleControl()
control.throttle = 0.3
vehicle.apply_control(control)


# Define camera
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
cam_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
camera = world.spawn_actor(rgb_camera_bp,
    cam_transform,
    attach_to=vehicle,
    attachment_type=carla.AttachmentType.Rigid)


# Save images + open window
def handle_image(disp, image):

    # Save images
    image.save_to_disk('output/%05d.png' % image.frame,
      carla.ColorConverter.Raw)

    org_array = np.frombuffer(image.raw_data, dtype=np.dtype('uint8'))
    array = np.reshape(org_array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:,:,::-1]
    array = array.swapaxes(0,1)
    surface = pygame.surfarray.make_surface(array)

    disp.blit(surface, (200,0))
    pygame.display.flip()


# Window settings
display = pygame.display.set_mode(
        (1200, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF
    )


# Update loop
camera.listen(lambda image: handle_image(display, image))


# Stop after 15 seconds
time.sleep(15)

camera.destroy()
vehicle.destroy()
