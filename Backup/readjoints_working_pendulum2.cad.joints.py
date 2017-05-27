# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""
import sys
import PyQt4.QtGui as qg
import numpy
import os
import yaml
import scipy.integrate
import scipy.linalg
import matplotlib.pyplot as plt
plt.ion()#interactive mode on

"""
import costum made functions
"""
import pynamics
from pynamics.variable_types import Constant
from pynamics.system import System
from support import ReadJoints
import support
from pynamics.output import Output
import animate

directory = './'
filename = 'pendulum2.cad.joints'
with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props = yaml.load(f)


system = System()
rigidbodies = []
for line,items in connections:
    for item in items:
        if not item in [item2.body for item2 in rigidbodies]:
            rigidbody = support.RigidBody.build(item)
            rigidbodies.append(rigidbody)

connections = [(line,tuple(sorted([b1.rigidbody,b2.rigidbody]))) for line,(b1,b2) in connections]
top = fixed_bodies[0]
N_rb = [body for body in rigidbodies if body.body==top][0]
N = N_rb.frame
system.set_newtonian(N)
O = 0*N.x
basis_vectors = [N.x,N.y,N.z]
unused = support.build_frames(rigidbodies,N_rb,connections,system,O,joint_props)
g = Constant('g',9.81,system)
system.addforcegravity(-g*N.z)
ini = [0, 0, 0.0, 0.0, 0, 100]
f,ma = system.getdynamics()

func1 = system.state_space_post_invert(f,ma)#original

animation_params = support.AnimationParameters(t_final=5)    
t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]

#==============================================================================
x,details=scipy.integrate.odeint(func1,ini,t,full_output=True)
#==============================================================================

print('calculating outputs..')
points1 = [[rb.particle.pCM.dot(bv) for bv in basis_vectors] for rb in rigidbodies]
output = Output(points1,system)
y = output.calc(x)
output3 = Output([N.getR(rb.frame) for rb in rigidbodies],system)
R = output3.calc(x)
R = R.reshape(-1,len(rigidbodies),3,3)
T = support.build_transformss(R,y)
bodies = [item.body for item in rigidbodies]    
readjoints = ReadJoints(bodies,T.tolist(),animation_params)
readjoints2 = ReadJoints(bodies,T[0:1,:].tolist(),animation_params)

pynamics.toc()

KE = system.KE
PE = system.getPEGravity(O) - system.getPESprings()
output2=Output([KE-PE],system)
y2 = output2.calc(x)
app = qg.QApplication(sys.argv)
animate.render(readjoints,show=True,save_files = False, render_video=True)
