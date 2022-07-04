#!/usr/bin/env python3
import time

import pygame

# -------------------------
# Initialization
# -------------------------

pygame.init()
pygame.joystick.init()  # Initialize the joysticks
clock = pygame.time.Clock()

joystick_count = pygame.joystick.get_count()
print('Found ' + str(joystick_count) + ' joysticks.')

# if joystick_count < 1:
#     print('No joysticks found. Terminating.')
#     exit(0)


joystick = pygame.joystick.Joystick(0)
joystick_name = joystick.get_name()
axes = joystick.get_numaxes()

print('Connected to joystick named ' + joystick_name)

while True:
    #
    # EVENT PROCESSING STEP
    #
    # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
    # JOYBUTTONUP, JOYHATMOTION
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")


# -------------------------
# Execution in cycle
# -------------------------
    try:
        jid = joystick.get_instance_id()
    except AttributeError:
        # get_instance_id() is an SDL2 method
        jid = joystick.get_id()
    print("Joystick {}".format(jid))

    # while True:
    # jid = joystick.get_instance_id()
    # print(jid)
    # axis = joystick.get_axis(1)
    # print(axis)
    # button = joystick.get_button(1)
    # print(button)

    # joystick = pygame.joystick.Joystick(0)
    # axis0 = joystick.get_axis(0)
    throttle = joystick.get_axis(2)
    #
    # print('Axis0=' + str(axis0) + '; Axis1=' + str(axis1))
    #     axes = joystick.get_numaxes()
    #     print("Number of axes: {}".format(axes))

        # for i in range(axes):
        #     axis = joystick.get_axis(i)
        #     print("Axis {} value: {:>6.3f}".format(i, axis))
        #
        # clock.tick(2)


# -------------------------
# Termination
# -------------------------

pygame.quit()
