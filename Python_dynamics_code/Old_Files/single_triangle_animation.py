# Python script to test 2D implementation of lines and dots as a preliminary step of robot visualization
'''
This script requires the following python libraries:
numpy
matplotlib (specifically pyplot and patches)
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def UpdateTriangle(ell, q1, q2):
    # Define the location of the bottom left vertex at 0,0
    p1 = np.array([0,0])
    # Calculate the angle from the x axis using the Law of Cosines
    theta = np.arccos((q1**2 + q2**2 - (ell - q1 - q2)**2)/(2*q1*q2))
    # p2 is the bottom right vertex
    p2 = p1 + np.array([q1,0])
    # p3 is the top vertex
    p3 = p1 + np.array([q2*np.cos(theta),q2*np.sin(theta)])
    p = np.array([p1,p2,p3])
    return p

def UpdatePlot(ell, ax, q1, p):
    # Clear the plot
    p1 = p[0]
    p2 = p[1]
    p3 = p[2]

    ax.cla()
    # Plot the triangle with dots at each vertex and lines connecting the vertices
    ax.plot([p1[0],p2[0]],[p1[1],p2[1]],'b-')
    ax.plot([p2[0],p3[0]],[p2[1],p3[1]],'r-')
    ax.plot([p3[0],p1[0]],[p3[1],p1[1]],'g-')
    ax.plot(p1[0],p1[1],'bo')
    ax.plot(p2[0],p2[1],'ro')
    ax.plot(p3[0],p3[1],'go')

    ellipse = Ellipse((q1/2,0), (ell-q1), np.sqrt(((ell-q1)/2)**2 - 1/4*q1**2)*2, angle=0, fill=False, edgecolor='blue')
    ax.add_patch(ellipse)

    plt.xlim(-ell/10, ell/2)
    plt.ylim(-ell/10, ell/2)
    plt.draw()
    plt.pause(0.03)


# https://www.thisiscarpentry.com/2013/09/06/drawing-an-ellipse-the-string-method/

# Equation of an ellipse centered at (h,k):
# (x-h)**2/a**2 + (y-k)**2/b**2 = 1
# Where b is 1/2 the width and a is 1/2 the height of the ellipse

if __name__ == '__main__':
    # For this case, the width of the ellipse is ell-q1 and the height is np.sin(theta)*q2*2
    # Create a triangle with dots at each vertex
    # Define the side lengths of the triangle
    ell = 5
    q1, q2 = ell/3, ell/3 # q1 is the base of the triangle, q2 is the other side adjacent to the leftmost vertex
    # Create an ellipse using the base dimension of the triangle
    # Parameters are: center location (x,y), width, height, angle of rotation, fill color, edge color
    ellipse = Ellipse((q1/2,0), (ell-q1), np.sqrt(((ell-q1)/2)**2 - 1/4*q1**2)*2, angle=0, fill=False, edgecolor='blue')

    # Plot the triangle with dots at each vertex and lines connecting the vertices
    fig, ax = plt.subplots()

    for i in range(75,150,1):
        q2 = i/300*(ell)
        q1 = q2
        # q2, q3 = (ell-q1)/2, (ell-q1)/2
        UpdatePlot(ell, ax, q1, UpdateTriangle(ell, q1, q2))

    plt.show()


