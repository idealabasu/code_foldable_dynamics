# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 11:38:12 2017

@author: Roozbeh Khodambashi
rkhodamb@asu.edu
"""
import sys
import PyQt5.QtGui as qg
import numpy
import sympy
import math
import os
import yaml
import scipy.integrate
import scipy.linalg
import matplotlib.pyplot as plt
import pickle
plt.ion()#interactive mode on
"""
import costum made functions
"""
import pynamics

from pynamics.variable_types import Constant
from pynamics.variable_types import Differentiable
from pynamics.system import System
from pynamics.frame import Frame
from support import ReadJoints
from support import RigidBody
import support
from pynamics.output import Output
import animate
from pynamics.particle import Particle
from sympy.functions import Abs
from pynamics.frame import Frame
from foldable_robotics.dynamics_info import DynamicsInfo
from pynamics.variable_types import Differentiable
from math import pi
system = System()

# Getting back the objects:
with open('objs.pickle','rb') as f:  # Python 3: open(..., 'rb')
    t, func1, ini, animation_params, rigidbodies, c1, c2, c3 = pickle.load(f)

#x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-8,atol=1e-8,full_output=True)#use without Baumgartes
x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-3,atol=1e-3,full_output=True,args=(1e2,1e1))#use with Baumgartes
readjoints = support.ReadJoints.build(x,animation_params,rigidbodies,system)
constraint_output=Output([c1,c2,c3],system)
cy = constraint_output.calc(x)
import idealab_tools.data_exchange.csv as csv
csv.write('output1.csv',numpy.c_[t,x])
plt.plot(cy)
#increment = ini[0]/10
#sum = increment
#for counter in range(0, 9):
##x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-8,atol=1e-8,full_output=True)#use without Baumgartes
#    ini = [sum, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0, 0, 0, 0, 0, 0]
#    x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-3,atol=1e-3,full_output=True,args=(1e2,1e1))#use with Baumgartes
#    sum = sum + increment
#
#    readjoints = support.ReadJoints.build(x,animation_params,rigidbodies,system)
#
#    constraint_output=Output([c1,c2,c3],system)
#    cy = constraint_output.calc(x)
#    import idealab_tools.data_exchange.csv as csv
#    csv.write('output'+str(sum)+'.csv',numpy.c_[t,x[:,0]])
#    plt.plot(cy)
pynamics.toc()

#KE = system.KE
#PE = system.getPEGravity(O) - system.getPESprings()



 
app = qg.QApplication(sys.argv)
animate.render(readjoints,material_properties,delete_images=True)
w=animate.animate(readjoints,material_properties)
sys.exit(app.exec_())


# obj0, obj1, obj2 are created here...



