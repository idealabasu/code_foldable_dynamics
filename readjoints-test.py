# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""
import sys
import PyQt5.QtGui as qg
import numpy
import os
import yaml
import scipy.integrate
import scipy.linalg
import matplotlib.pyplot as plt
plt.ion()#interactive mode on
import copy
"""
import costum made functions
"""
import popupcad
import pynamics
from pynamics.variable_types import Constant
from pynamics.variable_types import Differentiable
from pynamics.system import System
from pynamics.frame import Frame
from support_test import ReadJoints
from support_test import RigidBody
import support_test
from pynamics.output import Output
import animate


#pynamics.tic()
directory = './'
filename = 'five bar linkage3.cad.joints'
#filename = 'test.joints'
#filename = 'pendulum2.cad.joints'
with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props = yaml.load(f)


system = System()
L  = Constant('L' ,0.5,system)

rigidbodies = []
for line,items in connections: #we already have allbodies, we dont need to iterate over connections, items are elements of allbodies???
    for item in items:
        if not item in [item2.body for item2 in rigidbodies]:
            rigidbody = support_test.RigidBody.build(item)
            rigidbodies.append(rigidbody)

#additional_body = copy.deepcopy(rigidbodies[3])#the right wing
#rigidbodies.append(additional_body)
connections = [(line,tuple(sorted([b1.rigidbody,b2.rigidbody]))) for line,(b1,b2) in connections]#rebuilding connections with new rigid bodies created
top = fixed_bodies[0]
N_rb = [body for body in rigidbodies if body.body==top][0]#when we have already chosen the fixed_bodies[0] in previous line, why should we choose the [0] in here?? maybe because we want the same body from 'connections' instead of 'rigidbodies'
N = N_rb.frame
system.set_newtonian(N)
O = 0*N.x
basis_vectors = [N.x,N.y,N.z]

unused_connections, joint_props_dict, axis_list,parent_children, generations, counter = support_test.build_frames(rigidbodies,N_rb,connections,system,O,joint_props)
#additional_body = copy.copy(rigidbodies[3])

g = Constant('g',9.81,system)
system.addforcegravity(-g*N.z)
ini = [0]*len(system.state_variables())
f,ma = system.getdynamics()

#=========mycode=============
unused_parent = unused_connections[0][1][0]
unused_child = unused_connections[0][1][1]

counter = 0
for layer in unused_child.body.layerdef.layers:
        unused_child.body.layerdef.layers[counter].density = unused_child.body.layerdef.layers[counter].density/2
        counter = counter+1
unused_child_copy = copy.copy(unused_child)
#==============================================================================
# for layer in unused_childCopy.body.layerdef.layers:
#         unused_childCopy.body.layerdef.layers[counter].density = unused_childCopy.body.layerdef.layers[counter].density/2
#         counter = counter+1
#==============================================================================
#unused_childCopy = copy.deepcopy(unused_child)      
unused_line = unused_connections[0][0]
k,b,q0,lim_neg,lim_pos,joint_z = joint_props_dict[unused_line]
unused_points = numpy.c_[unused_line.exteriorpoints(),[joint_z,joint_z]]/popupcad.SI_length_scaling
unused_axis = unused_points[1] - unused_points[0]
l = (unused_axis.dot(unused_axis))**.5
unused_axis = unused_axis/l
unused_fixedaxis = unused_axis[0]*unused_parent.frame.x+unused_axis[1]*unused_parent.frame.y+unused_axis[2]*unused_parent.frame.z
ghost_frame = unused_child_copy.frame
x,x_d,x_dd = Differentiable(system)
   
unused_child_copy.frame.rotate_fixed_axis_directed(unused_parent.frame,unused_axis,x,system)
x_vec = ghost_frame.x
y_vec = ghost_frame.y
eq1 = [
       x_vec.dot(N.x)-1, 
       y_vec.dot(N.y)-1,
       unused_child_copy.vector_from_fixed(unused_child_copy.body.mass_properties()[2]).dot(N.x) - unused_child_copy.vector_from_fixed(unused_parent.body.mass_properties()[2]).dot(N.x),      
       unused_child_copy.vector_from_fixed(unused_child_copy.body.mass_properties()[2]).dot(N.y) - unused_child_copy.vector_from_fixed(unused_parent.body.mass_properties()[2]).dot(N.y) 
       ] 

#==============================================================================
# newvec = unused_parent.vector_from_fixed(unused_points[0])
# newvec1 = unused_child.vector_from_fixed(unused_points[0])
# newvec2 = unused_parent.vector_from_fixed(unused_points[1])
# newvec3 = unused_child.vector_from_fixed(unused_points[1])
#==============================================================================

#==============================================================================
# N = Frame('N')
# A = Frame('A')
# B = Frame('B')
# C = Frame('C')
# D = Frame('D')
# system.set_newtonian(N)
# A.rotate_fixed_axis_directed(N,axis_list[0],system.q[0][0],system)
# B.rotate_fixed_axis_directed(A,axis_list[2],system.q[0][2],system)
# C.rotate_fixed_axis_directed(N,axis_list[1],system.q[0][1],system)
# D.rotate_fixed_axis_directed(C,axis_list[3],system.q[0][3],system)
# 
# pNA=0*N.x
# pAB=pNA+lA*A.x
# pBC = pAB + lB*B.x
# pCtip = pBC + lC*C.x
#==============================================================================
#==============================================================================
# ghost_frame.x.dot(unused_child.frame.x)-1,
# ghost_frame.y.dot(unused_child.frame.y)-1,
# ghost_frame.z.dot(unused_child.frame.z)-1,
#==============================================================================
#==============================================================================
# eq1 = [
#        newvec.dot(newvec1)-newvec.dot(newvec), 
#        newvec2.dot(newvec3)-newvec2.dot(newvec2)              
#         ] 
#==============================================================================
#==============================================================================
# newvec = unused_parent.vector_from_fixed(unused_points[0])
# newvec1 = unused_child.vector_from_fixed(unused_points[0])
# eq1 = [newvec.dot(N.x) - newvec1.dot(N.x),
#        newvec.dot(N.y) - newvec1.dot(N.y),
#        newvec.dot(N.z) - newvec1.dot(N.z)
#         ] 
#==============================================================================
eq1_d = [system.derivative(item) for item in eq1]
eq1_dd = [(system.derivative(item)) for item in eq1_d]

func1 = system.state_space_post_invert(f, ma, eq1_dd)#original
#func1 = system.state_space_post_invert2(f,ma, eq1_dd, eq1_d, eq1, eq_active = [True, True])#constraint, the number of True should be equal to number of active constraints

animation_params = support_test.AnimationParameters(t_final=100)    
t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]
x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-12,atol=1e-12,hmin=1e-14)
print('calculating outputs..')
points1 = [[rb.particle.pCM.dot(bv) for bv in basis_vectors] for rb in rigidbodies]
output = Output(points1,system)
y = output.calc(x)
output3 = Output([N.getR(rb.frame) for rb in rigidbodies],system)
R = output3.calc(x)
R = R.reshape(-1,len(rigidbodies),3,3)
T = support_test.build_transformss(R,y)
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
animate.animate(readjoints)
#sys.exit(app.exec_())

#==============================================================================
# plt.plot(y[:,1,0],y[:,1,2])
# plt.plot(y[:,2,0],y[:,2,2])
# plt.plot(y[:,3,0],y[:,3,2])
# plt.plot(t,y2)
# plt.show()
# 
#==============================================================================
