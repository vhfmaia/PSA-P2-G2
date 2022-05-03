import carla
import time
import pygame
import numpy

camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')

#camera configuration
camera.set(bloom_intensity=0.675)
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)
camera.set(iso=100)
camera.set(gamma=2.2)
camera.set(sensor_tick=0.5)
camera.set(shutter_speed=60)

#Camera lens distortion attributes
camera.set(lens_circle_falloff=5.0)
camera.set(lens_circle_multiplier=0.0)
camera.set(lens_k=-1.0)
camera.set(lens_kcube=0.0)
camera.set_lens_size(x=0.08, y=0.08)

carla_settings.add_sensor(camera)
