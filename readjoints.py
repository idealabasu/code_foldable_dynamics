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


#pynamics.tic()
directory = './'
filename = 'five bar linkage3.cad.joints'
#filename = 'test.joints'
#filename = 'pendulum2.cad.joints'
with open(os.path.join(directory,filename),'r') as f:
    allbodies,connections,fixed_bodies,joint_props = yaml.load(f)

# ==============================================================================
#  a1 = allbodies[1]
#  a2 = allbodies[2]
#  a3 = allbodies[3]
#  
#  b1 = a1.mass_properties() #return volume_total,mass_total,center_of_mass,I
#  b2 = a2.mass_properties()
#  b3 = a3.mass_properties()
#  
#  print(b1)
#  print(b2)
#  print(b3)
#  
# ==============================================================================

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
ini = [.01]*len(system.state_variables())
f,ma = system.getdynamics()
#==============================================================================
# pCtip = 0
# vCtip = pCtip.time_derivative(N,system)
# eq1_d = [vCtip.dot(N.y)]
# eq1_dd=[(system.derivative(item)) for item in eq1_d]
# eq = eq1_dd
#==============================================================================

func1 = system.state_space_post_invert(f,ma)#original
#func1 = system.state_space_post_invert2(f,ma, eq_1dd, eq_1d, eq_1, eq_active = [True,True])#original

animation_params = support.AnimationParameters(t_final=5)    
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
readjoints2 = ReadJoints(bodies,T[0:1,:].tolist(),animation_params)

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
animate.animate(readjoints2)
#sys.exit(app.exec_())

#==============================================================================
# plt.plot(y[:,1,0],y[:,1,2])
# plt.plot(y[:,2,0],y[:,2,2])
# plt.plot(y[:,3,0],y[:,3,2])
# plt.plot(t,y2)
# plt.show()
# 
#==============================================================================
