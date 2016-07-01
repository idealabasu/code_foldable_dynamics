# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""
import pynamics
pynamics.tic()
print('setup')
from pynamics.variable_types import Constant
import numpy
import sympy
import os
from pynamics.system import System
import math
from support import ReadJoints
import yaml
#import time
import support
import scipy.integrate
import scipy.linalg
from pynamics.output import Output
import popupcad
from pynamics.frame import Frame
from pynamics.variable_types import Differentiable
from math import pi
import animate

directory = 'C:\\Users\\daukes\\desktop'
filename = 'pendulum2.cad.joints'

with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props = yaml.load(f)

sys = System()
g = Constant('g',9.81,sys)

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
sys.set_newtonian(N)
O = 0*N.x
basis_vectors = [N.x,N.y,N.z]
unused = support.build_frames(rigidbodies,N_rb,connections,sys,O,joint_props)
sys.addforcegravity(-g*N.z)
ini = [0]*len(sys.state_variables())
f,ma = sys.getdynamics()
func1 = sys.createsecondorderfunction2(f,ma)
animation_params = support.AnimationParameters()    
t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]
x,details=scipy.integrate.odeint(func1,ini,t,full_output=True)
print('calculating outputs..')
points1 = [[rb.particle.pCM.dot(bv) for bv in basis_vectors] for rb in rigidbodies]
output = Output(points1,sys)
y = output.calc(x)

output = Output([N.getR(rb.frame) for rb in rigidbodies],sys)
R = output.calc(x)
R = R.reshape(-1,len(rigidbodies),3,3)

T = support.build_transformss(R,y)

bodies = [item.body for item in rigidbodies]    

if __name__=='__main__':
    for body in bodies:
        del body.rigidbody

readjoints = ReadJoints(bodies,T.tolist(),animation_params)

#if __name__=='__main__':
#    import yaml
#    with open('rundata','w') as f1:
#        yaml.dump(readjoints,f1)

animate.render(readjoints,show=True,save_files = False, render_video=False)