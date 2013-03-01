from math import sqrt
import numpy as np
import matplotlib.pyplot as pl
import scipy
from scipy import optimize
from math import pi, sin, cos
from dynamic_graph import plug
from math import pi, cos
from dynamic_graph.tutorial import FeedbackController
from dynamic_graph.sot.dynamics import flexibility_f, flexibility_h
from dynamic_graph.sot.core import Kalman, Multiply_double_vector, \
    Add_of_vector
from dynamic_graph.sot.tools import Oscillator
from dynamic_graph.tutorial.simulator import Simulator

import sys, os
sys.path.append (os.getenv('DEVEL_DIR') + "/experiments/python")
from tools import getMaxima, fitFunction, fitSinusoid

simulator = Simulator (name = "robot", m = 57., zeta = .8077, kth = 1020,
                       kdth = 30., Iyy = 6.9417085929720743)

dt = 0.00001
startTime = 0
# Initial state
x0 = (0., 0., 0., 0.)
xi = 0
simulator.robot.state.value = x0

# read angular frequency and amplitude
with open ('./README', 'r') as readme:
    line = readme.readline ()
    omega = float (line [8:])
    line = readme.readline ()
    A = float (line [4:])

# simulate data
My = []
xiSot = []
comRs = []
duration = 400*pi/omega
dxi = 0
gain = 20
Dxi = []
D2xi = []
Theta = []
Dtheta = []
D2theta = []

for i in range (int (duration/dt)):
    t = dt*i
    xiRef = A * cos (omega*t)
    dxiRef = - A * omega * sin (omega*t)
    d2xiRef = - A * omega**2 * cos (omega*t)
    xi = simulator.robot.state.value [0]
    dxi = simulator.robot.state.value [2]
    Dxi.append (dxi)
    Theta.append (simulator.robot.state.value [1])
    Dtheta.append (simulator.robot.state.value [3])
    xiSot.append ((xiRef, 0., simulator.zeta))
    comRs.append ((xi, 0., simulator.zeta))
    my = simulator.robot.output.value [1]
    My.append (my)
    u = d2xiRef - 2*gain * (dxi -dxiRef) - gain**2 * (xi - xiRef)
    D2xi.append (u)
    simulator.robot.control.value = (u,)
    simulator.robot.control.time = i
    simulator.robot.incr (dt)
    simulator.robot.output.recompute (i)
    d2theta = (simulator.robot.state.value [3] - Dtheta [-1])/dt
    D2theta.append (d2theta)

with open ('./comRs.plot', 'w') as f:
    for i, com in zip (xrange (startTime, startTime + 1000000), comRs):
        f.write ("{0}\t{1}\t{2}\t{3}\n".format (i, com [0], com [1], com [2]))

maximaRs, magnitudeRs = getMaxima (map (lambda x: x[0], comRs), startTime)

# build Momentum signal as arrays
period = 2*pi/omega
nbPt = int (3*period/dt)
times = scipy.array (map (lambda i: i*dt, range (nbPt)))
t0 = maximaRs [100] - startTime
values = scipy.array (My [t0: t0 + nbPt])

A = magnitudeRs
m = simulator.m
g = simulator.g
zeta = simulator.zeta
Iyy = simulator.Iyy

nu = m*g + m*zeta*omega**2

fitfunc = fitFunction (zeta, nu, Iyy, A, omega)
errfunc = lambda p, x, y: fitfunc (p, x) - y # Distance to the target function

p0 = [0.,
      900.,
      0.]

# Fit sinusoid
p1, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

# Write result in a file
with open ('./My-fit-Iyy.plot', 'w') as f:
    for i in range (t0 + startTime, t0+startTime + len (times)):
        t = (i - (t0+startTime))*dt
        f.write ("{0}\t{1}\n".format (i, fitfunc (p1, t)))

with open ('./My.plot', 'w') as f:
    for i, my in zip (xrange (startTime, startTime + 1000000), My):
        f.write ("{0}\t{1}\n".format (i, my))

with open ('./theta.plot', 'w') as f:
    for i, th in zip (xrange (startTime, startTime + 1000000), Theta):
        f.write ("{0}\t{1}\n".format (i, th))

with open ('./dtheta.plot', 'w') as f:
    for i, dth in zip (xrange (startTime, startTime + 1000000), Dtheta):
        f.write ("{0}\t{1}\n".format (i, dth))

with open ('./d2theta.plot', 'w') as f:
    for i, d2th in zip (xrange (startTime, startTime + 1000000), D2theta):
        f.write ("{0}\t{1}\n".format (i, d2th))

# M (t) = B cos (omega t) + C sin (omega t)
kth = p1 [1]
kdth = p1 [2]

B = A*((zeta*nu - kth + Iyy*omega**2)*kth*nu - kdth**2*nu*omega**2)/((zeta*nu-kth+Iyy*omega**2)**2 + omega**2*kdth**2)
C = - A*(kdth*nu**2*zeta*omega + kdth*nu*Iyy*omega**3)/((zeta*nu-kth+Iyy*omega**2)**2 + omega**2*kdth**2)

fitfunc = fitSinusoid (omega)
errfunc = lambda p, x, y: fitfunc (p, x) - y # Distance to the target function
p0 = [0., B, C]
p2, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

# Write result in a file
with open ('./My-fit-Iyy2.plot', 'w') as f:
    for i in range (t0 + startTime, t0+startTime + len (times)):
        t = (i - (t0+startTime))*dt
        f.write ("{0}\t{1}\n".format (i, fitfunc (p2, t)))

# Fit dxi
values = scipy.array (Dxi [t0: t0 + nbPt])
p0 = [0, 0, -omega*A]
p3, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

# Fit d2xi
values = scipy.array (D2xi [t0: t0 + nbPt])
p0 = [0, -omega**2*A, 0]
p4, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

# Fit theta
values = scipy.array (Theta [t0: t0 + nbPt])
p0 = [0, A, 0]
p5, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

# Fit dtheta
values = scipy.array (Dtheta [t0: t0 + nbPt])
p0 = [0, omega*p5 [2], -omega*p5 [1]]
p6, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

# Fit d2theta
values = scipy.array (D2theta [t0: t0 + nbPt])
p0 = [0, -omega**2*p5 [1], -omega**2*p5 [2]]
p7, success = optimize.leastsq(errfunc, p0[:], args=(times, values))

toComplex = lambda p : complex (p [1], -p[2])
xi = complex (A, 0)
dxi = toComplex (p3)
d2xi = toComplex (p4)
th = toComplex (p5)
dth = toComplex (p6)
d2th = toComplex (p7)

with open ('./coefficient-Iyy', 'w') as f:
    f.write ("A = {0}\n".format (A))
    f.write ("B = {0}\n".format (B))
    f.write ("C = {0}\n".format (C))
    f.write ("kth = {0}\n".format (kth))
    f.write ("kdth = {0}\n".format (kdth))

print ("omega = {0}".format (omega))
print ("A = {0}".format (A))
print ("B = {0}".format (B))
print ("C = {0}".format (C))
print ("B2 = {0}".format (p2 [1]))
print ("C2 = {0}".format (p2 [2]))
print ("kth = {0}".format (kth))
print ("kdth = {0}".format (kdth))


