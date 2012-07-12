from math import sqrt
import numpy as np
import matplotlib.pyplot as pl
from math import pi, sin
from dynamic_graph import plug
from dynamic_graph.tutorial import TableCart, FeedbackController

cartHeight = .80
gravity = 9.81
mass = 56.
# define inverted pendulum
robot = TableCart ("robot")
robot.setCartMass (mass)
robot.setCartHeight (cartHeight)

# gains
kx0 = 1.
kx1 = 1.
kz = 0.5

# Initial state
robot.state.value = (1., 0.)

K = FeedbackController("K")

K.setComGain (kx0)
K.setComDotGain (kx1)
K.setZmpGain (kz)

plug(robot.state, K.state)
plug(robot.zmp, K.zmp)
plug(K.control, robot.control)

timeStep = 0.001

def play (nbSteps):
    timeSteps = []
    states = []
    velocities = []
    accelerations = []
    forces = []
    zmps = []
    controls = []
    tests = []

    # Loop over time and compute discretized state
    for x in xrange(nbSteps) :
        t = x*timeStep
        if t <= 1.:
            robot.force.value = sin (pi*t)
        timeSteps.append(t)
        states.append(robot.state.value)
        forces.append(robot.force.value)
        zmps.append(robot.zmp.value)
        controls.append (K.control.value)
        robot.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y1 = np.array (states).transpose ()
    y2 = np.array (forces)
    y3 = np.array (zmps)
    y4 = np.array (controls)

    fig  = pl.figure ()
    ax1 = fig.add_subplot (121)
    ax2 = fig.add_subplot (122)

    # plot configuration variables
    ax1.plot (x,y1[0])
    ax1.plot (x,y1[1])
    ax1.plot (x,y3)

    # plot velocity variables
    ax2.plot(x,y2)
    ax2.plot(x,y4)

    leg = ax1.legend(("x", "dx/dt", "zmp"))
    leg = ax2.legend(("f", "u"))

    pl.show()

if __name__ == '__main__' :
    play(3000)

