# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

import sys
import PyQt4.QtGui as qg
import PyQt4.QtCore as qc
from support import ReadJoints
import yaml
with open('rundata','r') as f:
    rundata = yaml.load(f)
    
import numpy
import pyqtgraph.opengl as pgo

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
w.opts['distance'] = 5000
w.opts['azimuth'] = -45
w.opts['elevation'] = 45
w.resize(640,480)


ii = 0

import os
if not os.path.exists('render/'):
    os.mkdir('render')

ee = numpy.array(rundata.ee)

#w.show()
#w.showMaximized()
#w.showFullScreen()
w.updateGL()

for ii in range(len(ee)):
    for jj,mi in enumerate(meshitems):
        tr = ee[ii,jj]
        tr =qg.QMatrix4x4(*tr.flatten().tolist())
        mi.setTransform(tr)
    w.updateGL()
    w.grabFrameBuffer().save('render/img_{0:04d}.png'.format(ii))
    print(ii)
#w.close()

import subprocess
if os.path.exists('render.mp4'):
    os.remove('render.mp4')

subprocess.call('"C:/program files/ffmpeg/bin/ffmpeg" -r 30 -i render/img_%04d.png -vcodec libxvid render.mp4')

import shutil
shutil.rmtree('render')
sys.exit(app.exec_())
