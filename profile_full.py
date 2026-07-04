import cProfile
import pstats

# Import the main script and profile just a few frames of the real loop!
import sys
import pygame
import time

sys.argv = ['test_autonomous_controller.py', '--simulator']

import test_autonomous_controller as tac
import Simulator.TurtleBotSim.turtlebot as tb

# Override the loop to only run for 50 frames
original_flip = pygame.display.flip
frame_count = 0
def mock_flip():
    global frame_count
    frame_count += 1
    if frame_count > 50:
        pygame.quit()
        sys.exit(0)
    original_flip()
pygame.display.flip = mock_flip

try:
    cProfile.run('tac.main()', 'profile_stats')
except SystemExit:
    pass

p = pstats.Stats('profile_stats')
p.sort_stats('time').print_stats(15)
