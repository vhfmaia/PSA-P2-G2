# Libraries Camera
import carla
import numpy as np
import pygame
import pygame.joystick

# Initiate Pygame and Joystick
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
Throttle = Brake = Steer = 0
done = False
# Get count of joysticks.
joystick_count = pygame.joystick.get_count()
joystick = pygame.joystick.Joystick(0)
joystick_name = joystick.get_name()
axes = joystick.get_numaxes()

def throttle():
    for event in pygame.event.get():
        False
    Throttle = round((1 - joystick.get_axis(2))/2, 4)

    return Throttle

def brake():
    for event in pygame.event.get():
        False
    Brake = round((1 - joystick.get_axis(3))/2, 4)

    return Brake


def steer():
    for event in pygame.event.get():
        False
    Steer = round((joystick.get_axis(0)), 4)

    return Steer

# # For each joystick:
# for i in range(joystick_count):
#     joystick = pygame.joystick.Joystick(i)
#     joystick.init()
#
# done = False
# while not done:
#     axes = joystick.get_numaxes()
#     rt = joystick.get_axis(5)
#     print (rt)
#     clock.tick(2)
#     done = False

# Connect to carla
client = carla.Client('localhost', 2000)
client.set_timeout(40.0)

# Change map
world = client.load_world("Town04")

# Change Weather
world.set_weather(carla.WeatherParameters.ClearNoon)

# Select vehicle
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]

# Start position
transform = carla.Transform()

transform.location.x = -490.8
transform.location.y = 229.1
transform.location.z = 1.5

transform.rotation.yaw = -89
transform.rotation.pitch = 0
transform.rotation.roll = 0

# Spawn vehicle at start position
vehicle = world.spawn_actor(vehicle_bp, transform)

# Spawn spectator
spectator = world.get_spectator()
sp_transform = carla.Transform(transform.location + carla.Location(z=30, x=-25),
                               carla.Rotation(yaw=90, pitch=-90))
spectator.set_transform(sp_transform)

# Move vehicle


# Define camera
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
cam_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
camera = world.spawn_actor(rgb_camera_bp,
                           cam_transform,
                           attach_to=vehicle,
                           attachment_type=carla.AttachmentType.Rigid)


# Image
def handle_image(disp, image):
    # Save image
    image.save_to_disk('output/%05d.png' % image.frame, carla.ColorConverter.Raw)

    # Handle image
    org_array = np.frombuffer(image.raw_data, dtype=np.dtype('uint8'))
    array = np.reshape(org_array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    array = array.swapaxes(0, 1)
    surface = pygame.surfarray.make_surface(array)

    # Update window
    disp.blit(surface, (200, 0))
    pygame.display.flip()


# Window settings
display = pygame.display.set_mode(
    (1200, 600),
    pygame.HWSURFACE | pygame.DOUBLEBUF
)

# Update
camera.listen(lambda image: handle_image(display, image))
while True:
    control = carla.VehicleControl()
    control.throttle = throttle()
    control.brake = brake()
    control.steer = steer()
    vehicle.apply_control(control)
    print(control.steer)

# Stop
camera.destroy()
vehicle.destroy()