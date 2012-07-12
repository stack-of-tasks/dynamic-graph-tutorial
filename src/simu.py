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

K = FeedbackController("K")
kx = 10.
omega2 = gravity/cartHeight
kz = .5*kx-sqrt(9*kx**2-8*omega2)/6

K.setComGain (kx)
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
            robot.force.value = (sin (pi*t),)
        timeSteps.append(t)
        states.append(robot.state.value)
        velocities.append (robot.velocity.value)
        accelerations.append (robot.acceleration.value)
        tests.append (robot.acceleration.value [0] +
                      omega2/kz*robot.velocity.value [0]+
                      omega2/kz*(kx - kz)*robot.state.value [0] -
                      robot.force.value [0]/mass)
        forces.append(robot.force.value)
        zmps.append(robot.zmp.value)
        controls.append (K.control.value)
        robot.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y1 = np.array (states).transpose ()
    y2 = np.array (forces).transpose ()
    y3 = np.array (zmps).transpose ()
    y4 = np.array (controls).transpose ()
    y5 = np.array (velocities).transpose ()
    y6 = np.array (accelerations).transpose ()

    fig  = pl.figure ()
    ax1 = fig.add_subplot (121)
    ax2 = fig.add_subplot (122)

    # plot configuration variables
    ax1.plot (x,y1[0])
    ax1.plot (x,y3[0])
    ax1.plot (x,tests)

    # plot velocity variables
    ax2.plot(x,y2[0])
    ax2.plot(x,y4[0])
    ax2.plot(x,y5[0])
    ax2.plot(x,y6[0])

    leg = ax1.legend(("x", "zmp", "test"))
    leg = ax2.legend(("f", "u", "dx", "ddx"))

    pl.show()

if __name__ == '__main__' :
    play(5)

