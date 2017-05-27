# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

from pynamics.variable_types import Differentiable
from pynamics.frame import Frame
from pynamics.particle import Particle
import numpy
import popupcad
from pynamics.vector import Vector

class ReadJoints(object):
    def __init__(self,rigidbodies,ee,animation_params):
        self.rigidbodies = rigidbodies
        self.ee = [[item.matrix().tolist() for item in item2] for item2 in ee]
        self.animation_params = animation_params
        
def vector_from_fixed(fixed_matrix,fixed_vector,new_matrix,frame):
    dx = new_matrix - fixed_matrix
    vec1 = Vector({frame:dx})
    vec2 = fixed_vector+vec1
    return vec2     

def add_spring_between_points(P1,P3,accounting,N_rb,k_stop,b):
    constraint1 = P1 - P3
    c1_d = constraint1.diff_in_parts(N_rb.frame,accounting)
    accounting.add_spring_force(k_stop,constraint1,c1_d)
#    accounting.addforce(-b*c1_d,c1_d)

def find_constraints(unused_connections):
    constraint_sets = []

    for line, bodies in unused_connections:
        points = line.exteriorpoints()
        points = numpy.c_[points,[0,0]]
        points = points/popupcad.SI_length_scaling
        
        v1 = bodies[0].vector_from_fixed(points[0])
        v2 = bodies[0].vector_from_fixed(points[1])
        v3 = bodies[1].vector_from_fixed(points[0])
        v4 = bodies[1].vector_from_fixed(points[1])
        constraint_sets.append([v1,v2,v3,v4])

    return constraint_sets
    
class RigidBody(object):
    def __init__(self,body,frame):
        self.body = body
        self.frame = frame
        self.frame.rigidbody = self
        self.body.rigidbody = self

    def set_fixed(self,point,vector):
        self.fixed_initial_coordinates = point.tolist()
        self.fixed_vector = vector

    def get_fixed(self):
        return numpy.array(self.fixed_initial_coordinates),self.fixed_vector

    def set_particle(self,particle):
        self.particle = particle
        
    @classmethod
    def build(cls,body):
        frame = Frame(str(body.id))
        new = cls(body,frame)
        return new

    def gen_info(rigidbody):
        volume_total,mass_total,center_of_mass,I = rigidbody.body.mass_properties()
#        layers = lam[:]
#        layer = layers[0].unary_union(layers)
#        areas = numpy.array([shape.area for shape in layer.geoms])
#        centroids = numpy.array([shape.centroid.coords[0] for shape in layer.geoms])
        
#        area = sum(areas)
#        centroid = (areas*centroids).sum(0)/area
#        center_of_mass /= popupcad.SI_length_scaling
#        volume_total /=popupcad.SI_length_scaling**3
        return volume_total,center_of_mass
        
    def vector_from_fixed(self,new_matrix):
        fixed_matrix,fixed_vector = self.get_fixed()
        vec = vector_from_fixed(fixed_matrix,fixed_vector,new_matrix,self.frame)
        return vec

    def __lt__(self,other):
        return self.body.id<other.body.id


class AnimationParameters(object):
    def __init__(self,t_initial=0,t_final=20,fps=30):
        self.t_initial = t_initial
        self.t_final = t_final
        self.fps = fps
        self.t_step = 1./fps

def build_frames(rigidbodies,N_rb,connections,accounting,O,joint_props):
    from math import pi
    parent_children,unused_connections,generations = characterize_tree(connections,rigidbodies,N_rb)    
    connections_rev = dict([(bodies,line) for line,bodies in connections])
    joint_props_dict = dict([(item,prop) for (item,bodies),prop in zip(connections,joint_props)])
    for generation in generations:
        for parent in generation:    
            for child in parent_children[parent]:
                line = connections_rev[tuple(sorted([parent,child]))]
                k,b,q0,lim_neg,lim_pos,joint_z = joint_props_dict[line]   
                
                points = numpy.c_[line.exteriorpoints(),[joint_z,joint_z]]/popupcad.SI_length_scaling
                axis = points[1] - points[0]
                l = (axis.dot(axis))**.5
                axis = axis/l
                fixedaxis = axis[0]*parent.frame.x+axis[1]*parent.frame.y+axis[2]*parent.frame.z

                x,x_d,x_dd = Differentiable(accounting)
                child.frame.rotate_fixed_axis_directed(parent.frame,axis,x,accounting)
                
                w = parent.frame.getw_(child.frame)
                t_damper = -b*w
                spring_stretch = (x-(q0*pi/180))*fixedaxis
                accounting.addforce(t_damper,w)
                accounting.add_spring_force(k,spring_stretch,w)
    child_velocities(N_rb,O,numpy.array([0,0,0]),N_rb,accounting,connections_rev,joint_props_dict)
    
    return unused_connections
                                
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
            for line,bodies in connections[:]:#the first loop picks the first connection and corresponding bodies from connections
                if parent in bodies:
                    connections.remove((line,bodies))
                    ii = bodies.index(parent)                
                    child = bodies[1-ii]#in two body pairs, if the first one is parent, ii is 0 and child will be the index 1 (second body), otherwise ii would be 1 and child will be the first
                    if child in allchildren:
                        unused_connections.append((line,bodies))
                    else:
                        parent_children[parent].append(child)
                        allchildren.append(child)
                        children.append(child)
                        
        generations.append(searchqueue)
        searchqueue = children
        children = []#children is reseted in the beginnig of the while loop, is tere a need to rest it here again?
    return parent_children,unused_connections,generations
    
def child_velocities(parent,referencepoint,reference_coord,N_rb,accounting,connections_rev,joint_props_dict):
    parent.set_fixed(reference_coord,referencepoint)
    volume_total,center_of_mass = parent.gen_info()
#    centroid = numpy.r_[centroid,[0]]
    newvec = parent.vector_from_fixed(center_of_mass)
    p = Particle(accounting,newvec,1)#fix actual mass instead of 1, use body instead with inertia calc.
    parent.set_particle(p)
    
    for child in parent.frame.children:
        child = child.rigidbody
        line = connections_rev[tuple(sorted([parent,child]))]
        k,b,q0,lim_neg,lim_pos,joint_z = joint_props_dict[line]   

        points = numpy.c_[line.exteriorpoints(),[joint_z,joint_z]]/popupcad.SI_length_scaling
        newvec = parent.vector_from_fixed(points[0])
        child_velocities(child,newvec,points[0],N_rb,accounting,connections_rev,joint_props_dict)
        
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
    
def build_transformss(Rx,y):
    from pyqtgraph import Transform3D
    import PyQt4.QtGui as qg
    transformss = []
    for ii,aa in enumerate(Rx):
        cc = []
        for jj,bb in enumerate(aa):
            bb=bb.T
            T1 = numpy.eye(4)
            T1[:3,3] = -y[0,jj]*1000
            T2 = numpy.eye(4)
            T2[:3,:3] = bb
            T3 = numpy.eye(4)
            T3[:3,3] = y[ii,jj]*1000
            T = T3.dot(T2.dot(T1))
            tr = Transform3D()
            for kk,item in enumerate(T):
                tr.setRow(kk,qg.QVector4D(*item))
            cc.append(tr)
        transformss.append(cc)
    transformss = numpy.array(transformss)    
    return transformss
    
    