# Python script to test 2D implementation of lines and dots as a preliminary step of robot visualization
'''
This script requires the following python libraries:
numpy
matplotlib (specifically pyplot and patches)
keyboard
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import keyboard as kb
from single_triangle_animation import UpdateTriangle, UpdatePlot

# Define the total tube length
ell = 10
# Define the starting side lengths of the triangle
q1, q2 = ell/3, ell/3 # q1 is the base of the triangle, q2 is the other side adjacent to the leftmost vertex

fig, ax = plt.subplots()
UpdatePlot(ell, ax, q1, UpdateTriangle(ell, q1, q2))

plt.draw()
keyPress = 'a'
print("Use the arrow keys to move the triangle. Press 'q' to quit.")
while keyPress != 'q':
    keyPress = kb.read_key()    # Reads the key pressed by the user and stores it as the string ('str') equivalent
    if keyPress == 'up':
        q2 += 0.025
    elif keyPress == 'down':
        q2 -= 0.025
    elif keyPress == 'left':
        q1 -= 0.025
    elif keyPress == 'right':
        q1 += 0.025
    UpdatePlot(ell, ax, q1, UpdateTriangle(ell, q1, q2))

# Remove all keypresses from the register
kb.unhook_all()
# close the figure
plt.close()


