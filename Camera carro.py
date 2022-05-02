camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(camera)
