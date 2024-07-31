import matplotlib.pyplot as plt
import numpy as np
import nodeParam as P
from model_1_dynamics import NodeDynamics
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import os
import imageio_ffmpeg as ffmpeg
os.environ['PATH'] += os.pathsep + ffmpeg.get_ffmpeg_exe()


fig, ax = plt.subplots()
line1, = ax.plot([], [], 'bo-', lw=2)
line2, = ax.plot([], [], 'bo-', lw=2)
anchor1, = ax.plot([], [], 'ro', lw=2)  # Anchor for spring-damper 1
anchor2, = ax.plot([], [], 'ro', lw=2)  # Anchor for spring-damper 2
ax.set_aspect('equal')
ax.set_xlim(-1, 4)
ax.set_ylim(-1, 4)
ax.grid()

truss = NodeDynamics()

# Initialization function
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    anchor1.set_data([P.ell, 0])
    anchor2.set_data([0.0, P.ell])
    return line1, line2, anchor1, anchor2

# Update function for animation
def update(frame):
    F = 0.0
    truss.update(F)
    x, y = truss.h()
    # print(x, y)
    line1.set_data([x[0], P.ell], [y[0], 0.0])
    line2.set_data([x[0], 0.0], [y[0], P.ell])
    return line1, line2, anchor1, anchor2

def save_animation(dynamic_animation):
    f = r"C:/Users/stowel22/Desktop/animation.mp4"
    FFMpegWriter = animation.writers['ffmpeg']
    writer = FFMpegWriter(fps=30, metadata=dict(artist='Me'), bitrate=1800)
    dynamic_animation.save(f, writer=writer)

dynamic_animation = FuncAnimation(fig, update, frames=P.n_steps, init_func=init, blit=True, interval=1000*P.Ts)
plt.title("Truss Animation")
plt.xlabel("x")
plt.ylabel("y")

# Save the animation (may need to change the 'interval' parameter in dynamic_animcation to be 1 to get good frame rate)
# save_animation(dynamic_animation)
plt.show()