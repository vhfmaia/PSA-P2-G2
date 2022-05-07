import carla
import time
import numpy as np
import pygame
import random

client = carla.client('localhost',2000)
client.set_timeout(2.0)
world=client.load_wold("Town02")

world.set_weather(carla.WeatherParameters.ClearNoon)

bp=world.get_blueprint_library()
vehicle_bp=bp.filter('vehicle.tesla.model3') [0]
print(vehicle_bp)

spawn_point = random.choice(world.get_map().get_spawn_points())

vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
#actor_list.append(vehicle)

transform=carla.Transform()

transform.location.x=220
transform.location.y=-1.6
transform.location.z=1.85

transform.location.yaw=180
transform.location.pitch=0
transform.location.roll=0

vehicle=world.spaw_actor(vehicle_bp, transform)

spectator=world.get_spectator()
sp_transform=carla.Transform(transform.location + carla.Location(z=30, x=-25))
    carla.Rotation(yaw=90, pitch=-90)
spectator.set_tranform(sp_transform)

control=carla.VehicleControl()
control.throttle=0.3
vehicle.apply_control(control)

# rgb_camera_bp=world.get_blueprint_library().find('sensor.camera.rgb)')
# rgb_camera_bp.set_attribute("image_size_X", f"{IM_WIDTH}")
# rgb_camera_bp.set_attribute("image_size_Y", f"{IM_HEIGHT}")

cam_transform=carla.Transform(carla.Location(x=0.8, z=1.7))
Camera=world.spaw_actor(rgb_camera_bp,
                 cam_transform,
                 attach_to=vehicle,
                 attachment_type=carla.AttachmentType.Rigid)

def handle_image(image):
    #image.save_to_disk('output/%05d.png' % image.frame, carla.ColorConvert.Raw)
    org_array=np.frombuffer(image.raw_data, dtype=np.disp('uint8'))
    array=np.reshape(org_array, (image.height, image.width, 4))
    array=array[:, :, :3]
    array = array[:, :,::-1]
    array = array.swapaxes(0,1)
    surface=pygame.surfarray.make_surface(array)

    display.blit(surface, (200,0))
    pygame.display.flip()

display= pygame.display.set_mode(
        (1200, 600)
        pygame.HWSURFACE | pygame.DOUBLEBUF)

rgb_camera_bp.listen(lambda image: handle_image(display, image))

time.sleep(15)

rgb_camera_bp.destroy()
vehicle.destroy()

# camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')
#
# #camera configuration
# camera.set(bloom_intensity=0.675)
# camera.set(FOV=90.0)
# camera.set_image_size(800, 600)
# camera.set_position(x=0.30, y=0, z=1.30)
# camera.set_rotation(pitch=0, yaw=0, roll=0)
# camera.set(iso=100)
# camera.set(gamma=2.2)
# camera.set(sensor_tick=0.5)
# camera.set(shutter_speed=60)
#
# #Camera lens distortion attributes
# camera.set(lens_circle_falloff=5.0)
# camera.set(lens_circle_multiplier=0.0)
# camera.set(lens_k=-1.0)
# camera.set(lens_kcube=0.0)
# camera.set_lens_size(x=0.08, y=0.08)
#
# carla_settings.add_sensor(camera)
