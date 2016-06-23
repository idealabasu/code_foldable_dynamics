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

#t = sympy.Symbol('t')
#t0 = time.time()

#directory = 'C:\\Users\\danaukes\\popupCAD_files\\designs'
#directory = 'C:\\Users\\danaukes\\desktop'
directory = 'C:\\Users\\daukes\\desktop'
#directory = 'C:\\Users\\danaukes\\dropbox'
#directory = 'C:\\Users\\danb0b\\dropbox'
#filename = 'doublepend.cad.joints'
#filename = '225654992.cad.joints'
#filename = 'closed chain.cad.joints'
filename = '257464192.cad.joints'
#filename = 'twolayer.cad.joints'
#filename = 'triple_pendulum3.cad.joints'
#filename = 'broken.cad.joints'
with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props = yaml.load(f)

sys = System()
g = Constant('g',9.81,sys)
k = Constant('k',1e3,sys)
b = Constant('b',1e1,sys)

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
searchqueue = [top]

pynamics.toc()
#pynamics.tic()
print('building frames')
unused = support.build_frames(rigidbodies,N_rb,connections,sys,O,joint_props)
pynamics.toc()
#pynamics.tic()
print('fpoints')

pynamics.toc()
##pynamics.tic()
print('calculating constraints')
constraintsets = support.find_constraints(unused)
#support.add_spring_between_points(v1,v3,sys,N_rb,k_stop)
#support.add_spring_between_points(v2,v4,sys,N_rb,k_stop)
constraints = []

import popupcad
from pynamics.frame import Frame
from pynamics.variable_types import Differentiable
from math import pi

#for line,(body1,body2) in unused:
#    joint_props_dict = dict([(item,prop) for (item,bodies),prop in zip(connections,joint_props)])
#    k,b,q0,lim_neg,lim_pos = joint_props_dict[line]                
#    points = numpy.c_[line.exteriorpoints(),[0,0]]/popupcad.SI_length_scaling
#    axis = points[1] - points[0]
#    l = (axis.dot(axis))**.5
#    axis = axis/l
#    fixedaxis = axis[0]*body1.frame.x+axis[1]*body1.frame.y+axis[2]*body1.frame.z
#    x,x_d,x_dd = Differentiable(sys)
#    redundant_frame = Frame()
#    redundant_frame.rotate_fixed_axis_directed(body1.frame,axis,x,sys)
#    w = body1.frame.getw_(redundant_frame)
#    t_damper = -b*w
#    spring_stretch = (x-(q0*pi/180))*fixedaxis
#    sys.addforce(t_damper,w)
#    sys.add_spring_force(k,spring_stretch,w)
#    constraints.append(redundant_frame.x.dot(body2.frame.x)-1)
#    constraints.append(redundant_frame.y.dot(body2.frame.y)-1)
#    constraints.append(redundant_frame.z.dot(body2.frame.z)-1)
#
#zero = sympy.Matrix(constraints)
#constraints_d2 = [sys.derivative(item) for item in constraints]
#constraints_d3 = sympy.Matrix(constraints_d2)
#if len(constraints)>0:
#    J = constraints_d3.jacobian(sys.get_q(1))
#    fJ = sympy.lambdify(sys.state_variables(),J)



#constraints.append(v1)
#for v1,v2,v3,v4 in constraintsets:
#    constraints.append(v1-v3)
#    constraints.append(v2-v4)
#bv2 = [N.x, N.y, N.z]
#zero = [item.dot(bv) for item in constraints for bv in bv2]
#zero = sympy.Matrix(zero)
#constraints_d1 = [item.diff_in_parts(N,sys) for item in constraints]
##constraints_d2 = [item.dot(bv) for item in constraints_d1 for bv in basis_vectors]
#constraints_d2 = [item.dot(bv) for item in constraints_d1 for bv in bv2]
#constraints_d3 = sympy.Matrix(constraints_d2)
#if len(constraints)>0:
#    J = constraints_d3.jacobian(sys.get_q(1))
#    fJ = sympy.lambdify(sys.state_variables(),J)

pynamics.toc()
#pynamics.tic()
print('adding gravity')

sys.addforcegravity(-g*N.z)

ini = [0]*len(sys.state_variables())
pynamics.toc()
#pynamics.tic()
print('solving dynamics...')
f,ma = sys.getdynamics()

pynamics.toc()
#pynamics.tic()
print('creating second order function...')
    
if len(constraints)>0:
    func1 = sys.createsecondorderfunction6(f,ma,fJ,zero,sympy.Matrix(constraints_d2))
else:
    func1 = sys.createsecondorderfunction2(f,ma)

pynamics.toc()
#pynamics.tic()
print('integrating...')
animation_params = support.AnimationParameters()    
t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]
x,details=scipy.integrate.odeint(func1,ini,t,full_output=True)
#x=scipy.integrate.odeint(func1,ini,t)
pynamics.toc()
#pynamics.tic()c
print('calculating outputs..')


points1 = [[rb.particle.pCM.dot(bv) for bv in basis_vectors] for rb in rigidbodies]
output = Output(points1,sys)
y = output.calc(x)

output = Output([N.getR(rb.frame) for rb in rigidbodies],sys)
R = output.calc(x)
R = R.reshape(-1,len(rigidbodies),3,3)


#v1_d = (v1-v3).diff_in_parts(N,sys)
#points1 = [[v.dot(bv) for bv in basis_vectors] for v in constraints+constraints_d1]
#output = Output(points1,sys)
#vout = output.calc(x)


T = support.build_transformss(R,y)

#for rigidbody in rigidbodies:
#    del rigidbody.fixed_vector 
#    del rigidbody.fixed_initial_coordinates
#    del rigidbody.body.rigidbody

bodies = [item.body for item in rigidbodies]    

if __name__=='__main__':
#    import matplotlib.pyplot as plt
#    support.plot(t,x,y)
#    plt.figure()
#    if len(constraints)>0:
#        vout = vout.reshape((vout.shape[0],-1))
#        plt.plot(t,vout[:,:3])        
#        plt.figure()
#        plt.plot(t,vout[:,3:])        
    for body in bodies:
        del body.rigidbody

readjoints = ReadJoints(bodies,T.tolist(),animation_params.t_step)

if __name__=='__main__':
    import yaml
    with open('rundata','w') as f1:
        yaml.dump(readjoints,f1)


pynamics.toc()

#import pynamics.funkify
#variables = sys.get_q(0)+sys.get_q(1)+sys.get_q(2)
#f = pynamics.funkify.cythonify(ma[-1],variables)
#out = f(*[1 for item in variables])
##c = sympy.lambdify(variables,a)
#plt.figure(2)
#plt.xlabel('time(s)')
#plt.ylabel('rotation(rad)')
#plt.legend(['q1','q2','q3'])
#plt.show()