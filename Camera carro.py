camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')
#camera configuratuion
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

carla_settings.add_sensor(camera)
