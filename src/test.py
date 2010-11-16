import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph.tutorial as dgt
import dynamic_graph.signal_base as dgsb

# define inverted pendulum
a = dgt.InvertedPendulum("IP")
a.setCartMass(1.0)
a.setPendulumMass(1.0)
a.setPendulumLength(1.0)

# Set value of state signal
a.signal('state').value = '[4](0.0,0.01,0.0,0.0)'

timeStep = 0.01
timeSteps = []
values = []

# Loop over time and compute discretized state values
for x in xrange(10000) :
    t = x*timeStep
    timeSteps.append(t)
    values.append(dgsb.stringToTuple(a.signal('state').value))
    a.incr(timeStep)

# Convert into numpy array
x = np.array(timeSteps)
y = np.array(values).transpose()

fig  = pl.figure()
ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)

# plot configuration variables
ax1.plot(x,y[0])
ax1.plot(x,y[1])

# plot velocity variables
ax2.plot(x,y[2])
ax2.plot(x,y[3])

leg = ax1.legend(("x", "theta"))
leg = ax2.legend(("dx", "dtheta"))

pl.show()

