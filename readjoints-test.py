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
from pynamics.particle import Particle


#pynamics.tic()
directory = './'
#directory ='C:\\Users\\rkhodamb\\Desktop'
#filename = 'five bar linkage3.cad.joints'
#filename = 'five bar linkage3_Torgue1.cad.joints'

#filename = 'pendulum2.cad.joints'


#filename = 'newMechanism.cad.joints'
#filename = 'newMechanism1.cad.joints'
#filename = 'Prototype_1.joints'
#filename = 'fiveBar.cad.joints'
filename = 'sixBar1.cad.joints'

with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props = yaml.load(f)

#new_laminate = unused_child.body.copy(identical=False)

system = System()
#L  = Constant('L' ,0.5,system)

rigidbodies = []
for line,items in connections: #we already have allbodies, we dont need to iterate over connections, items are elements of allbodies???
    for item in items:
        if not item in [item2.body for item2 in rigidbodies]:
            rigidbody = support_test.RigidBody.build(item)#item is a laminate type 
            rigidbodies.append(rigidbody)

connections = [(line,tuple(sorted([b1.rigidbody,b2.rigidbody]))) for line,(b1,b2) in connections]#rebuilding connections with new "rigid bodies" created instead of "laminates"
top = fixed_bodies[0]
N_rb = [body for body in rigidbodies if body.body==top][0]#when we have already chosen the fixed_bodies[0] in previous line, why should we choose the [0] in here?? maybe because we want the same body from 'connections' instead of 'rigidbodies'
N = N_rb.frame
system.set_newtonian(N)
O = 0*N.x
basis_vectors = [N.x,N.y,N.z]

new_rigid_body,unused_child, generations = support_test.build_frames(rigidbodies,N_rb,connections,system,O,joint_props)

g = Constant('g',9.81,system)
system.addforcegravity(-g*N.z)


#==============================================================================
# initial conditions should later be updated from the cad file
#==============================================================================
#ini = [.01]*len(system.state_variables())

#ini = [0.000001, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0]#fiveBar.cad.joints
ini = [0.000001, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0]#sixBar1.cad.joints
#ini = [0.000001, 0, 0.0, 0.0, 0, 0, 0, 0]#newmechanism.cad.joints
#ini = [0.0001, 0.000, 0.000, 0.000, 0, 0, .0001, 0, 0, 0, 0, 0, 0, 0]#newmechanism1.cad.joints
#==============================================================================
f,ma = system.getdynamics()

#=========mycode=============

p1 = new_rigid_body.vector_from_fixed((1,2,3))
p2 = unused_child.vector_from_fixed((1,2,3))
p3 = new_rigid_body.vector_from_fixed((4,5,6))
p4 = unused_child.vector_from_fixed((4,5,6))
p5 = new_rigid_body.vector_from_fixed((1,0,0))
p6 = unused_child.vector_from_fixed((1,0,0))

l1 = p1-p2
l2 = p3-p4
l3 = p5-p6



ghost_frame = new_rigid_body.frame#this is the copy of the body that has been created using the half of the mass and inertia of the unused bodies  (unused_child or unused_parent)
unused_child_frame = unused_child.frame
eq1 = [
#==============================================================================
#         l1.dot(l1),
#         l2.dot(l2),
#         l3.dot(l3),
#==============================================================================
#these settings are best for 5 bar mechanism with no singularity (test.cad.joints)      
#      ghost_frame.y.dot(N.y)-unused_child_frame.y.dot(N.y),
#      ghost_frame.z.dot(N.z)-unused_child_frame.z.dot(N.z),
#      new_rigid_body.vector_from_fixed((1,2,3)).dot(N.y) - unused_child.vector_from_fixed((1,2,3)).dot(N.y),
#      new_rigid_body.vector_from_fixed((3,2,1)).dot(N.z) - unused_child.vector_from_fixed((3,2,1)).dot(N.z),
# #these settings work best for 4 bar mechanism  (newMechanism.cad.joints)    
#        ghost_frame.z.dot(N.z)-unused_child_frame.z.dot(N.z),
#        new_rigid_body.vector_from_fixed((1,2,1)).dot(N.z) - unused_child.vector_from_fixed((1,2,1)).dot(N.z),  
#        new_rigid_body.vector_from_fixed((3,2,1)).dot(N.z) - unused_child.vector_from_fixed((3,2,1)).dot(N.z),
#==============================================================================
#these settins work for both 5 bar and 4 bar mechanisms mentioned above, tolerance should be e^-8 for 5 bar to solve       
       ghost_frame.z.dot(N.z)-unused_child_frame.z.dot(N.z),
       new_rigid_body.vector_from_fixed((1,2,1)).dot(N.z) - unused_child.vector_from_fixed((1,2,1)).dot(N.z),
       new_rigid_body.vector_from_fixed((3,2,1)).dot(N.z) - unused_child.vector_from_fixed((3,2,1)).dot(N.z),
#==============================================================================
#        #ghost_frame.x.dot(unused_child_frame.x)-1, 
#        #ghost_frame.y.dot(unused_child_frame.y)-1,
#        #ghost_frame.z.dot(unused_child_frame.z)-1,
#               
#        #new_rigid_body.particle.pCM.dot(N.x)-unused_child.particle.pCM.dot(N.x),#for this constraint, the ini matrix can be zero
#        #new_rigid_body.particle.pCM.dot(N.y)-unused_child.particle.pCM.dot(N.y),       
#        #new_rigid_body.particle.pCM.dot(N.z)-unused_child.particle.pCM.dot(N.z),
#        
#        #ghost_frame.x.dot(N.x)-unused_child_frame.x.dot(N.x),
#        #ghost_frame.x.dot(N.y)-unused_child_frame.x.dot(N.y),
#        #ghost_frame.x.dot(N.z)-unused_child_frame.x.dot(N.z),
#        
#        #ghost_frame.y.dot(N.y)-unused_child_frame.y.dot(N.y),
#        ghost_frame.z.dot(N.z)-unused_child_frame.z.dot(N.z),
#        #ghost_frame.z.dot(N.y)-unused_child_frame.z.dot(N.y),
#        #ghost_frame.z.dot(N.x)-unused_child_frame.z.dot(N.x),
#        #ghost_frame.z.dot(unused_child_frame.z)-1,
#        
#        #new_rigid_body.vector_from_fixed((1,2,1)).dot(N.x) - unused_child.vector_from_fixed((1,2,1)).dot(N.x),
#        #new_rigid_body.vector_from_fixed((1,2,3)).dot(N.y) - unused_child.vector_from_fixed((1,2,3)).dot(N.y),   
#        new_rigid_body.vector_from_fixed((1,2,1)).dot(N.z) - unused_child.vector_from_fixed((1,2,1)).dot(N.z),  
#        new_rigid_body.vector_from_fixed((3,2,1)).dot(N.z) - unused_child.vector_from_fixed((3,2,1)).dot(N.z),  
#        #new_rigid_body.vector_from_fixed(new_rigid_body.body.mass_properties()[2]).dot(N.x) - unused_child.vector_from_fixed(unused_child.body.mass_properties()[2]).dot(N.x)      
#        #new_rigid_body.vector_from_fixed(new_rigid_body.body.mass_properties()[2]).dot(N.z) - unused_child.vector_from_fixed(unused_child.body.mass_properties()[2]).dot(N.z)          
#        #generations[3][0].vector_from_fixed(generations[3][0].body.mass_properties()[2]).dot(N.y) - generations[3][0].vector_from_fixed(generations[2][0].body.mass_properties()[2]).dot(N.y)     
#==============================================================================
       ] 

eq1_d = [system.derivative(item) for item in eq1]
eq1_dd = [(system.derivative(item)) for item in eq1_d]

#func1 = system.state_space_post_invert(f, ma)#no constraints
func1,A_full = system.state_space_post_invert(f, ma, eq1_dd)#constraints
#func1 = system.state_space_post_invert2(f,ma, eq1_dd, eq1_d, eq1, eq_active = [True, True, True])#Baumgartes constraints, the number of True should be equal to number of active constraints

animation_params = support_test.AnimationParameters(t_final=5,fps=1000)    
t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]

x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-8,atol=1e-8,full_output=True)#use without Baumgartes
#x,details=scipy.integrate.odeint(func1,ini,t,rtol=1e-5,atol=1e-5,hmin=1e-14,full_output=1,args=(1e8,1e3))#use with Baumgartes

print('calculating outputs..')
points1 = [[rb.particle.pCM.dot(bv) for bv in basis_vectors] for rb in rigidbodies]
output = Output(points1,system)
y = output.calc(x)
output3 = Output([N.getR(rb.frame) for rb in rigidbodies],system)
output4 = Output(A_full,system)
R = output3.calc(x)
R = R.reshape(-1,len(rigidbodies),3,3)
T = support_test.build_transformss(R,y)
bodies = [item.body for item in rigidbodies]    
readjoints = ReadJoints(bodies,T.tolist(),animation_params)

pynamics.toc()

#KE = system.KE
#PE = system.getPEGravity(O) - system.getPESprings()
#output2=Output([KE-PE],system)
#y2 = output2.calc(x)

app = qg.QApplication(sys.argv)
animate.render(readjoints,show=True,save_files = True, render_video=True)

#animate.render(readjoints,show=True)
animate.animate(readjoints)
#sys.exit(app.exec_())

