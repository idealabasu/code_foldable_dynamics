# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""
import pynamics
pynamics.script_mode = False
from pynamics.dyadic import Dyadic
import numpy
from pynamics.vector import Vector
import PyQt5.QtGui as qg
from pynamics.output import Output
from pynamics.variable_types import Constant
Constant.typestring = 'c'
Constant.typeformat = '{0}{1:d}'
from pynamics.variable_types import Differentiable
Differentiable.typestring = 'q'
Differentiable.typeformat = '{0}{1:d}'
from pynamics.variable_types import Variable
Variable.typestring = 'v'
Variable.typeformat = '{0}{1:d}'
from pynamics.frame import Frame
Frame.typestring = 'F'
Frame.typeformat = '{0}{1:d}'
from pynamics.body import Body
Body.typestring = 'B'
Body.typeformat = '{0}{1:d}'


class MassConstant(Constant):
    typestring = 'm'
class SpringConstant(Constant):
    typestring = 'k'
class DamperConstant(Constant):
    typestring = 'b'
class IniConstant(Constant):
    typestring = 'qini'

class ReadJoints(object):
    @classmethod
    def build(cls,x,animation_params,rigidbodies,system):
    #    print('calculating outputs..')
        N = system.newtonian
        basis_vectors = [N.x,N.y,N.z]
        points1 = [[rb.body.pCM.dot(bv) for bv in basis_vectors] for rb in rigidbodies]
        output = Output(points1,system)
        y0 = output.calc(numpy.array([[0]*len(system.get_state_variables())]))
        
        y = output.calc(x)
        output3 = Output([N.getR(rb.frame) for rb in rigidbodies],system)
        #output4 = Output(A_full,system)
        R = output3.calc(x)
        R = R.reshape(-1,len(rigidbodies),3,3)
        T = build_transformss(R,y,y0)
        bodies = [item.laminate for item in rigidbodies]    
        readjoints = cls(bodies,T.tolist(),animation_params)
        return readjoints

    def __init__(self,rigidbodies,ee,animation_params):
        self.rigidbodies = rigidbodies
        self.ee = [[item.matrix().tolist() for item in item2] for item2 in ee]
        self.animation_params = animation_params
        
def vector_from_fixed(fixed_matrix,fixed_vector,new_matrix,frame):
    dx = new_matrix - fixed_matrix
    vec1 = Vector({frame:dx})
    vec2 = fixed_vector+vec1
    return vec2     

#def add_spring_between_points(P1,P3,pynamics_system,N_rb,k_stop,b):
#    constraint1 = P1 - P3
#    c1_d = constraint1.diff_in_parts(N_rb.frame,pynamics_system)
#    pynamics_system.add_spring_force(k_stop,constraint1,c1_d)
##    pynamics_system.addforce(-b*c1_d,c1_d)

#def find_constraints(unused_connections):
#    constraint_sets = []
#
#    for line, bodies in unused_connections:
#        points = line.exteriorpoints()
#        points = numpy.c_[points,[0,0]]
#        
#        v1 = bodies[0].vector_from_fixed(points[0])
#        v2 = bodies[0].vector_from_fixed(points[1])
#        v3 = bodies[1].vector_from_fixed(points[0])
#        v4 = bodies[1].vector_from_fixed(points[1])
#        constraint_sets.append([v1,v2,v3,v4])
#
#    return constraint_sets
    
class RigidBody(object):
    def __init__(self,laminate,frame,material_prop):
        self.laminate = laminate
        self.material_prop = material_prop
        self.frame = frame
        self.frame.rigidbody = self
        self.laminate.rigidbody = self

    def set_fixed(self,point,vector):
        self.fixed_initial_coordinates = point.tolist()
        self.fixed_vector = vector

    def get_fixed(self):
        return numpy.array(self.fixed_initial_coordinates),self.fixed_vector

    def set_body(self,body):
        self.body = body

    @classmethod
    def build(cls,laminate, material_prop):
        frame = Frame()
        new = cls(laminate,frame,material_prop)
        return new

    def gen_info(self):
        mass_total,volume_total,center_of_mass,I = self.laminate.mass_properties(self.material_prop)
#        layers = lam[:]
#        layer = layers[0].unary_union(layers)
#        areas = numpy.array([shape.area for shape in layer.geoms])
#        centroids = numpy.array([shape.centroid.coords[0] for shape in layer.geoms])
        
#        area = sum(areas)
#        centroid = (areas*centroids).sum(0)/area
#        center_of_mass /= popupcad.SI_length_scaling
#        volume_total /=popupcad.SI_length_scaling**3
        return volume_total,mass_total,center_of_mass,I
        
    def vector_from_fixed(self,new_matrix):
        fixed_matrix,fixed_vector = self.get_fixed()
        vec = vector_from_fixed(fixed_matrix,fixed_vector,new_matrix,self.frame)
        return vec

    def __lt__(self,other):
        return self.laminate.id<other.laminate.id


class AnimationParameters(object):
    def __init__(self,t_initial=0,t_final=20,fps=30):
        self.t_initial = t_initial
        self.t_final = t_final
        self.fps = fps
        self.t_step = 1./fps

def build_frames(rigidbodies,N_rb,connections,pynamics_system,O,material_properties,torqueFunctions):
    from sympy import pi
    rigidbodies = rigidbodies[:]
    connections = connections[:]
    parent_children,unused_connections,generations = characterize_tree(connections,rigidbodies,N_rb) 
#==============================================================================
#     if unused_connections==[]:
#         unused_connections = [connections[-1] ]
#==============================================================================
    same_bodies = []
    new_rigid_body = []
    unused_child = []
    for connection in unused_connections:
        unused_joint = connection[0]
        unused_parent = connection[1][0]
        unused_child = connection[1][1]
        joint_prop = connection [2]
        #modify the mass properties of the unused_child and new_rigid_body
        for prop in unused_child.material_prop:
            prop.density /=2
        new_laminate = unused_child.laminate.copy(identical=False)
        mp = [item2.copy() for item2 in unused_child.material_prop]
        new_rigid_body = RigidBody.build(new_laminate,mp)
        rigidbodies.append(new_rigid_body)
        connections.append((unused_joint,(unused_parent,new_rigid_body),joint_prop))
        #connections[unused_joint][1] = new_rigid_body
    
        parent_children[unused_parent].append(new_rigid_body)#child of unused_parent=new_rigid_body
        parent_children[new_rigid_body]=[]
        same_bodies.append((unused_child,new_rigid_body))
        #=======
        #modify generation, adding new rigid body to generation below unused parent
        searchqueue = [new_rigid_body]
        generations.append(searchqueue)
        #========       
    connections_rev = dict([(bodies,(line,joint_props)) for line,bodies,joint_props in connections])
    connections_rev.update(dict([(tuple(bodies[::-1]),(line,joint_props)) for line,bodies,joint_props in connections]))
    axis_list = []
    
    counter = 0
    for generation in generations:
        for parent in generation:    
            
            for child in parent_children[parent]:
                line,joint_props = connections_rev[(parent,child)]
                
                k = joint_props.stiffness
                b = joint_props.damping
                q0 = joint_props.preload
#                lim_neg = joint_props.limit_neg
#                lim_pos = joint_props.limit_pos
                joint_z = joint_props.z_pos
                
                ck = SpringConstant(k,system = pynamics_system)
                cb = DamperConstant(b,system = pynamics_system)
                cq0 = IniConstant(q0,system = pynamics_system)
#                clim_neg = Constant(lim_neg,system = pynamics_system)
#                clim_pos = Constant(lim_pos,system = pynamics_system)
#                cjoint_z = Constant(joint_z,system = pynamics_system)
                
                points = numpy.c_[line,[joint_z,joint_z]]
                axis = points[1] - points[0]
                l = (axis.dot(axis))**.5
                axis = axis/l
                axis_list.append(axis)
                fixedaxis = axis[0]*parent.frame.x+axis[1]*parent.frame.y+axis[2]*parent.frame.z

                x,x_d,x_dd = Differentiable(system = pynamics_system)
                child.frame.rotate_fixed_axis_directed(parent.frame,axis,x,pynamics_system)
                
                w = parent.frame.getw_(child.frame)
                t_damper = -cb*w
                spring_stretch = (x-(cq0*pi/180))*fixedaxis
                pynamics_system.addforce(t_damper,w)
                pynamics_system.add_spring_force1(ck,spring_stretch,w)
                pynamics_system.addforce(torqueFunctions[counter]*fixedaxis,w) 
                counter =counter+1
    child_velocities(N_rb,O,numpy.array([0,0,0]),N_rb,pynamics_system,connections_rev,material_properties)
    #modify mass here of both unused_child and new_rigid body using same_bodies as a reference of the bodies which need to be changed.
#==============================================================================
#     unused_connections_rev = dict([(bodies,line) for line,bodies in unused_connections])
#     unused_parent = unused_connections[0][1][0]
#     unused_child = unused_connections[0][1][1]
#     unused_line = unused_connections[0][0]
#     k,b,q0,lim_neg,lim_pos,joint_z = joint_props_dict[unused_line]
#==============================================================================
    
#==============================================================================
#     unused_points = numpy.c_[unused_line.exteriorpoints(),[joint_z,joint_z]]/popupcad.SI_length_scaling
#     unused_axis = unused_points[1] - unused_points[0]
#     l = (unused_axis.dot(unused_axis))**.5
#     unused_axis = unused_axis/l
#     unused_fixedaxis = unused_axis[0]*unused_parent.frame.x+unused_axis[1]*unused_parent.frame.y+unused_axis[2]*unused_parent.frame.z
# 
#     x,x_d,x_dd = Differentiable(pynamics_system)
#     ghost_frame = Frame('ghost')     
#     ghost_frame.rotate_fixed_axis_directed(unused_parent.frame,axis,x,pynamics_system)
# #    ghost_frame.rotate_fixed_axis_directed(unused_child.frame,axis,x,pynamics_system)
#==============================================================================
    
    return  new_rigid_body,unused_child, generations
    #return  generations   
                             
def characterize_tree(connections,rigidbodies,N_rb):
    searchqueue = [N_rb]
    connections = connections[:]
    parent_children = {}
    unused_connections = []
    allchildren = []
    generations = []
    for body in rigidbodies:
        parent_children[body]=[]#parent and children of each body in rigidbodies
    while not not connections:
        children = []
        for parent in searchqueue:# the first loop picks the first fixed body from rigidbodies (which is in searchqeue)
            for line,bodies,joint_props in connections[:]:#the first loop picks the first connection and corresponding bodies from connections
                if parent in bodies:
                    connections.remove((line,bodies,joint_props))
                    ii = bodies.index(parent)                
                    child = bodies[1-ii]#in two body pairs, if the first one is parent, ii is 0 and child will be the index 1 (second body), otherwise ii would be 1 and child will be the first
                    if child in allchildren:
                        unused_connections.append((line,(parent,child),joint_props))
                        #parent_children[parent].append(child)# I added this line to close the last connection as well, it has to be tested to see if it works for all the cases
                    else:
                        parent_children[parent].append(child)
                        allchildren.append(child)
                        children.append(child)
                        
        generations.append(searchqueue)
        searchqueue = children
        children = []#children is reseted in the beginnig of the while loop, is there a need to reset it here again?
    return parent_children,unused_connections,generations
    
def child_velocities(parent,referencepoint,reference_coord,N_rb,pynamics_system,connections_rev,material_properties):
    parent.set_fixed(reference_coord,referencepoint)
    volume_total,mass_total,center_of_mass,I = parent.gen_info()
#    centroid = numpy.r_[centroid,[0]]

    cm = MassConstant(mass_total,system = pynamics_system)
    I_dyadic = Dyadic.build(parent.frame,I[0,0],I[1,1],I[2,2],I[0,1],I[1,2],I[2,0])

    newvec = parent.vector_from_fixed(center_of_mass)
#    p = Particle(newvec,1,pynamics_system)
    b = Body(None,parent.frame,newvec,cm,I_dyadic,pynamics_system)
    parent.set_body(b)
    
    for child in parent.frame.children:
        child = child.rigidbody
        line,joint_props = connections_rev[(parent,child)]

        k = joint_props.stiffness
        b = joint_props.damping
        q0 = joint_props.preload
        lim_neg = joint_props.limit_neg
        lim_pos = joint_props.limit_pos
        joint_z = joint_props.z_pos
                
        points = numpy.c_[line,[joint_z,joint_z]]
        newvec = parent.vector_from_fixed(points[0])
        child_velocities(child,newvec,points[0],N_rb,pynamics_system,connections_rev,material_properties)
        
def plot(t,x,y):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for item in y.transpose(1,0,2):
        ax.plot(item[:,0],item[:,1],zs = item[:,2])
    plt.show()
    plt.figure()
    plt.plot(t,x[:,0:3])        
    
def build_transformss(Rx,y,y0):
    from pyqtgraph import Transform3D
    transformss = []
    for ii,aa in enumerate(Rx):
        cc = []
        for jj,bb in enumerate(aa):
            bb=bb.T
            T1 = numpy.eye(4)
            T1[:3,3] = -y0[jj]
            T2 = numpy.eye(4)
            T2[:3,:3] = bb
            T3 = numpy.eye(4)
            T3[:3,3] = y[ii,jj]
            T = T3.dot(T2.dot(T1))
            tr = Transform3D()
            for kk,item in enumerate(T):
                tr.setRow(kk,qg.QVector4D(*item))
            cc.append(tr)
        transformss.append(cc)
    transformss = numpy.array(transformss)    
    return transformss
    
    