# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

import sys
import PyQt4.QtGui as qg
import PyQt4.QtCore as qc
#import readjoints
#rundata = readjoints.readjoints

from support import ReadJoints
import yaml
with open('rundata','r') as f:
    rundata = yaml.load(f)
    
import numpy
#from pyqtgraph.opengl import GLViewWidget
import pyqtgraph.opengl as pgo
from pyqtgraph import Transform3D

class ViewWidget(pgo.GLViewWidget):
    def __init__(self):
        super(ViewWidget,self).__init__()
        pass


app = qg.QApplication(sys.argv)
w = ViewWidget()    
w.setBackgroundColor(1,1,1,1)

def gen_mesh_item(body):
    colors = []
    all_points = []
    all_triangles = []
    
    for layer in body.layers():
        z = body.layerdef.z_values[layer]
        for geom in body.geoms[layer]:
            cdt = geom.triangles_inner()
            triangles = [[(point.x,point.y,z*1000) for point in triangle.points_] for triangle in cdt.GetTriangles()]        
            points = list(set([point for triangle in triangles for point in triangle]))
            all_points.extend(points)
            triangles2 = [[all_points.index(point) for point in tri] for tri in triangles]
            all_triangles.extend(triangles2)
            colors.extend([layer.color]*len(points))
            
    all_points = numpy.array(all_points)/1000
    all_triangles = numpy.array(all_triangles)
    meshitem = pgo.GLMeshItem(vertexes=all_points, faces=all_triangles, vertexColors=colors,smooth=True)
    return meshitem
    
meshitems = [gen_mesh_item(body) for body in rundata.rigidbodies]
[w.addItem(meshitem) for meshitem in meshitems]
centerpoint = qg.QVector3D(3.5,-1,1)
w.opts['center'] = centerpoint
w.opts['distance'] = 1000
w.opts['azimuth'] = -45
w.opts['elevation'] = 45
#w.opts['azimuth'] = 0
#w.opts['elevation'] = 0
w.resize(640,480)


#mi = meshitems[3]
#tr = Transform3D()
#tr.translate(3000,0,0)
#tr.rotate(5,0,1,0)
#tr.translate(-3000,0,0)

#tr.translate(2500,0,0)
#mi.setTransform(tr)

w.show()
#w.showMaximized()
#w.showFullScreen()

ii = 0

import os
if not os.path.exists('render/'):
    os.mkdir('render')
ee = numpy.array(rundata.ee)
def update(t,w):
    global ii
    if ii<len(ee):
        for jj,mi in enumerate(meshitems):
            tr = ee[ii,jj]
            tr =qg.QMatrix4x4(*tr.flatten().tolist())
            mi.setTransform(tr)
        ii+=1
        w.grabFrameBuffer().save('render/{0:04d}.png'.format(ii))
    else:
        t.stop()
        w.showNormal()
#    tr = Transform3D()
#    tr.translate(3000,0,0)
#    tr.rotate(5,0,1,0)
#    tr.translate(-3000,0,0)
#    tr.setRow(0,qg.QVector4D(0.996195, 0.000000, 0.087156, 11.415906))
#    qg.QVector4D(0.000000, 1.000000, 0.000000, 0.000000)
#    mi.applyTransform(tr,True)

t = qc.QTimer()
t.timeout.connect(lambda:update(t,w))
t.start(rundata.t_step*1000)

sys.exit(app.exec_())
#import makemovie