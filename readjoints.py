# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

import sys
import PyQt4.QtGui as qg
import pynamics
from pynamics.variable_types import Constant
import numpy
import sympy
import os
from pynamics.system import System
from support import ReadJoints
import yaml
import support
import scipy.integrate
import scipy.linalg
from pynamics.output import Output
import animate
import matplotlib.pyplot as plt
plt.ion()
from pynamics.frame import Frame

pynamics.tic()
#directory = './'
directory = 'C:\\Users\\daukes\\code\\foldable_robotics\\python\\tests'
from pynamics.variable_types import Differentiable
from math import pi
#filename = 'pendulum2.cad.joints'
filename = 'test.yaml'
with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props,thickness,density = yaml.load(f)
from foldable_robotics.laminate import Laminate
allbodies = [Laminate.import_dict(item) for item in allbodies]
allbodies_dict = dict([(item.id,support.RigidBody.build(item)) for item in allbodies])
system = System()
rigidbodies =allbodies_dict.values()
k = Constant('k',1e3,system)
b = Constant('b',1e1,system)
g = Constant('g',9.81,system)


connections = [(line,tuple(sorted([allbodies_dict[b1],allbodies_dict[b2]]))) for line,(b1,b2) in connections]
top = fixed_bodies[0]
N_rb = allbodies_dict[top]
N = N_rb.frame
system.set_newtonian(N)
O = 0*N.x
basis_vectors = [N.x,N.y,N.z]

unused = support.build_frames(rigidbodies,N_rb,connections,system,O,joint_props,thickness,density)
constraints = []

for line,(body1,body2) in unused:
    k,b,q0,lim_neg,lim_pos,joint_z = joint_props[line]                
    points = numpy.c_[line,[joint_z,joint_z]]
    axis = points[1] - points[0]
    l = (axis.dot(axis))**.5
    axis = axis/l
    fixedaxis = axis[0]*body1.frame.x+axis[1]*body1.frame.y+axis[2]*body1.frame.z
    x,x_d,x_dd = Differentiable(system)
    redundant_frame = Frame()
    redundant_frame.rotate_fixed_axis_directed(body1.frame,axis,x,system)
#    w = body1.frame.getw_(redundant_frame)
#    t_damper = -b*w
#    spring_stretch = (x-(q0*pi/180))*fixedaxis
#    system.addforce(t_damper,w)
#    system.add_spring_force(k,spring_stretch,w)
    constraints.append(redundant_frame.x.dot(body2.frame.x)-1)
    constraints.append(redundant_frame.y.dot(body2.frame.y)-1)
    constraints.append(redundant_frame.z.dot(body2.frame.z)-1)
    
    
eq1_dd=[system.derivative(system.derivative(item)) for item in constraints]
eq = eq1_dd
    
    

system.addforcegravity(-g*N.z)

ini = [0]*len(system.state_variables())
f,ma = system.getdynamics()
func1 = system.state_space_post_invert(f,ma)

q_dyn = system.get_q(2)[:-3]
q_con = system.get_q(2)[-3:]

animation_params = support.AnimationParameters(t_final=5)    
func1 = system.state_space_post_invert(f,ma,eq)

t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]
x,details=scipy.integrate.odeint(func1,ini,t,full_output=True)
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

pynamics.toc()

KE = system.KE
PE = system.getPEGravity(O) - system.getPESprings()
output2=Output([KE-PE],system)
y2 = output2.calc(x)

#if __name__=='__main__':
#    import yaml
#    for body in bodies:
#        del body.rigidbody
#    with open('rundata','w') as f1:
#        yaml.dump(readjoints,f1)
#
app = qg.QApplication(sys.argv)
#animate.render(readjoints,show=False,save_files = False, render_video=True)
animate.animate(readjoints,thickness)
sys.exit(app.exec_())

#plt.plot(y[:,1,0],y[:,1,2])
#plt.plot(y[:,2,0],y[:,2,2])
#plt.plot(y[:,3,0],y[:,3,2])
#plt.plot(t,y2)
#plt.show()
