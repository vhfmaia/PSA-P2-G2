import pygame

pygame.init()

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates.
clock = pygame.time.Clock()

# Initialize the joysticks.
pygame.joystick.init()

# -------- Main Program Loop -----------
while not done:
    #
    # EVENT PROCESSING STEP
    #
    # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
    # JOYBUTTONUP, JOYHATMOTION
    for event in pygame.event.get(): # User did something.
            done = False # We're doing this forever :)

    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()


    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()


        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
    axes = joystick.get_numaxes()
    # print horizontal
    axis0 = joystick.get_axis(0)
    print(axis0)

    # print vertical
    axis1 = joystick.get_axis(1)
    print(axis1)

    # print lt
    axis2 = joystick.get_axis(2)
    print(axis2)

    # print button rt
    axis5 = joystick.get_axis(5)
    print(axis5)

    # print button lb
    button4 = joystick.get_button(4)
    print(button4)

    # print button rb
    button5 = joystick.get_button(5)
    print(button5)

    # Limit to 20 frames per second.
    clock.tick(2)

pygame.quit()