# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
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
plt.ion()#interactive mode on
import copy
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
import pynamics.integration
directory = './designs'
#directory = 'C:\\Users\\danaukes\\Dropbox (Personal)\\mohammad'
#directory ='C:\\Users\\rkhodamb\\Desktop'
#filename = 'five bar linkage3.cad.joints'
#filename = 'five bar linkage3_Torgue1.cad.joints'
#directory = './'
#directory = 'C:\\Users\\daukes\\code\\foldable_robotics\\python\\tests'
#directory = 'C:\\Users\\danaukes\\code\\code_foldable_robotics\\python\\tests'
#directory = 'C:\\Users\\daukes\\desktop'
from pynamics.variable_types import Differentiable
from math import pi
#filename = 'pendulum2.cad.joints'
#filename = 'pendulum2.cad.joints'
#filename = 'dynamics-info.yaml'

#filename = 'newMechanism.cad.joints'
#filename = 'newMechanism1.cad.joints'
#filename = 'Prototype_1.joints'
#filename = 'fiveBar.cad.joints'
#filename = 'sixBar1.cad.joints'
filename = '4th_Design.cad1.joints'
#filename = 'test.cad.joints'
#filename = 'W_3.5.cad.joints'
with open(os.path.join(directory,filename),'r') as f:
    d = yaml.load(f,Loader=yaml.FullLoader)
from foldable_robotics.laminate import Laminate

#material_properties = d.material_properties

allbodies = [Laminate.import_dict(item) for item in d.connected_items]
allbodies_dict  = {}
for item in allbodies:
    mp = [item2.copy() for item2 in d.material_properties]
    allbodies_dict[item.id] = support.RigidBody.build(item,mp) 
rigidbodies = [allbodies_dict[item.id] for item in allbodies]
system = System()


connections = [(line,tuple(sorted([allbodies_dict[b1],allbodies_dict[b2]])),joint_prop) for line,(b1,b2),joint_prop in d.connections]
top = d.newtonian_ids[0]
N_rb = allbodies_dict[top]
N = N_rb.frame
system.set_newtonian(N)
O = 0*N.x
basis_vectors = [N.x,N.y,N.z]
#torqueFunctions = [0,0,0,0,10*sympy.sin(5*sympy.pi*system.t),0]#sin function
#torqueFunctions = [0,0,0,0,0,0]#zero

def stepFunction(time):
    if time <1:    
        return 0
    else:    
        return 1
       
#aaa =round(.5*(1+(system.t-2)/(Abs(system.t-2)+0.000000001)))
#bbb = round(0.5*(1+(system.t-8)/(Abs(system.t-8)+0.000000001)))
#torqueFunctions = [0,0,0,0,1*system.t,0]#ramp function
#torqueFunctions = [0,0,0,0,2*((Abs(system.t-2)+(system.t-2))/(2*Abs(system.t-2))-(Abs(system.t-8)+(system.t-8))/(2*Abs(system.t-8))),0]#step function
#torqueFunctions = [0,0,0,0,20*((.5*(1+(system.t-20)/(Abs(system.t-20)+0.000000001)))-(.5*(1+(system.t-30)/(Abs(system.t-30)+0.000000001)))),0]#step function
torqueFunctions = [0]*len(connections)

new_rigid_body,unused_child, generations = support.build_frames(rigidbodies,N_rb,connections,system,O,d.material_properties,torqueFunctions)

#
g = Constant(9.81,'g',system)
system.addforcegravity(-g*N.z)


#==============================================================================
# initial conditions should later be updated from the cad file
#==============================================================================
#ini = [0.3623,0 ]#*len(system.get_state_variables())
#ini = [0]*len(system.state_variables())
#ini = [0, 0, 0.0, 0.0, 0, 100]#
#ini = [0.000001, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0]#fiveBar.cad.joints
ini = [1, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0, 0, 0, 0, 0, 0]#sixBar1.cad.joints
#ini = [0.000001, 0, 0.0, 0.0, 0, 0, 0, 0]#newmechanism.cad.joints
#ini = [0.0001, 0.000, 0.000, 0.000, 0, 0, .0001, 0, 0, 0, 0, 0, 0, 0]#newmechanism1.cad.joints
#==============================================================================

f,ma = system.getdynamics()

#=========mycode=============
if new_rigid_body== []:
    func1 = system.state_space_post_invert(f, ma)#no constraints    
        
else:
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
    
    v1 = new_rigid_body.vector_from_fixed((1,2,1))- unused_child.vector_from_fixed((1,2,1))
    v2 = new_rigid_body.vector_from_fixed((3,2,1))- unused_child.vector_from_fixed((3,2,1))
    v3 = new_rigid_body.vector_from_fixed((1,4,1))- unused_child.vector_from_fixed((1,4,1))
    
    c1 = v1.dot(v1)
    c2 = v2.dot(v2)
    c3 = v3.dot(v3)
    
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
    c1,
    c2,
    c3,
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


    #func1 = system.state_space_post_invert(f, ma, eq1_dd)#constraints
    func1 = system.state_space_post_invert2(f,ma, eq1_dd, eq1_d, eq1, eq_active = [True, True, True])#Baumgartes constraints, the number of True should be equal to number of active constraints

#animation_params = support_test.AnimationParameters(t_final=5)#,fps=1000)    
#t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]
animation_params = support.AnimationParameters(t_final= 10,fps=30)#,fps=1000)    
t = numpy.r_[animation_params.t_initial:animation_params.t_final:animation_params.t_step]

#x,details=pynamics.integration.integrate_odeint(func1,ini,t,rtol=1e-8,atol=1e-8,full_output=True)#use without Baumgartes
x,details=pynamics.integration.integrate_odeint(func1,ini,t,rtol=1e-3,atol=1e-3,full_output=True,args=({'alpha':1e2,'beta':1e1,'constants':system.constant_values},))#use with Baumgartes

readjoints = support.ReadJoints.build(x,animation_params,rigidbodies,system)

pynamics.toc()

#KE = system.get_KE()
#PE = system.getPEGravity(O) - system.getPESprings()
constraint_output=Output([c1,c2,c3],system)
cy = constraint_output.calc(x)

import idealab_tools.data_exchange.csv as csv
csv.write('output1.csv',numpy.c_[t,x[:,0]])
plt.plot(cy)
 
app = qg.QApplication(sys.argv)
animate.render(readjoints,d.material_properties,delete_images=True)
w=animate.animate(readjoints,d.material_properties)
sys.exit(app.exec_())