import numpy as np
import matplotlib.pyplot as pl
from math import pi, sin, cos, sqrt
from dynamic_graph.tutorial import TableCart

class Simulator (object):
    g = 9.81
    def __init__ (self, name, m, zeta, kth, kdth, Iyy):
        self.name = name
        self.m = m
        self.zeta = zeta
        self.kth = kth
        self.kdth = kdth
        self.Iyy = Iyy
        self.robot = TableCart (name)
        self.robot.setCartMass (m)
        self.robot.setCartHeight (zeta)
        self.robot.setStiffness (kth)
        self.robot.setViscosity (kdth)
        self.robot.setMomentOfInertia (Iyy)

    def findRoots (self, gain):
        zeta = self.zeta
        kth = self.kth
        kdth = self.kdth
        m = self.m
        g = self.g
        A = np.matrix ([[0,0,1,0],
                        [0,0,0,1],
                        [0,0,0,0],
                        [-g/zeta**2, (m*g*zeta-kth)/(m*zeta**2),0,0]])
        B = np.matrix ([[0],[0],[1],[1/zeta]])
        C = np.matrix (gain)
        cp = np.poly (A-B*C)
        return np.roots (cp)
    
    def computeOffset (self, gain):
        zeta = self.zeta
        kth = self.kth
        kdth = self.kdth
        m = self.m
        g = self.g
        
        coef = g/(zeta**2*(gain [1]*(-gain [0]/zeta-g/zeta**2)-gain [0]*((g*m*zeta-kth)/(m*zeta**2)-gain [1]/zeta)))
        return [gain [1]*coef, -kth * gains [0]*coef]

    def computeGains (self, alphas):
        zeta = self.zeta
        kth = self.kth
        kdth = self.kdth
        m = self.m
        g = self.g

        alpha1 = alphas [0]
        alpha2 = alphas [1]
        alpha3 = alphas [2]
        alpha4 = alphas [3]
        # Get coefficients of characteristic polynomial
        a0 = alpha1*alpha2*alpha3*alpha4
        a1 = alpha2*alpha3*alpha4 + alpha1*alpha3*alpha4 + \
            alpha1*alpha2*alpha4 + alpha1*alpha2*alpha3
        a2 = alpha3*alpha4 + alpha2*alpha4 + alpha2*alpha3 + \
            alpha1*alpha4 +  alpha1*alpha3 + alpha1*alpha2
        a3 = alpha1 + alpha2 + alpha3 + alpha4
        # Compute feedback gains
        mu = kth/(m*zeta) - g
        L = np.matrix ([[0,0,zeta,1],
                        [zeta,1,kdth/(m*zeta),0],
                        [kdth/(m*zeta),0,mu,-g/zeta],
                        [mu,-g/zeta,0,0]])
        v = np.matrix ([[a3*zeta-kdth/(m*zeta)],
                        [a2*zeta - mu],
                        [a1*zeta],
                        [a0*zeta]])
        gains = np.linalg.solve (L, v)
        return tuple ([gains.item (i,0) for i in range (4)])

    def computeNumerator (self):
        zeta = self.zeta
        kth = self.kth
        kdth = self.kdth
        m = self.m
        g = self.g
        Iyy = self.Iyy

        return ((m*zeta**2 + Iyy)*kth*m*zeta - kdth**2*m*zeta,
                2*g*kth*m**2*zeta**2+kth*m*g*Iyy-kth**2*m*zeta-kdth**2*m*g,
                kth*m*g*(m*g*zeta - kth))
