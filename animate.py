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
import shutil
import os
import numpy
import pyqtgraph.opengl as pgo
import subprocess

class ViewWidget(pgo.GLViewWidget):
    def __init__(self):
        super(ViewWidget,self).__init__()
        pass

def gen_mesh_item(body):
    colors = []
    all_points = []
    all_triangles = []
    
    for layer in body.layers():
        z = body.layerdef.z_values[layer]
        for geom in body.geoms[layer]:
            cdt = geom.triangles_inner()
            triangles = [[(point.x,point.y,z) for point in triangle.points_] for triangle in cdt.GetTriangles()]        
            points = list(set([point for triangle in triangles for point in triangle]))
            all_points.extend(points)
            triangles2 = [[all_points.index(point) for point in tri] for tri in triangles]
            all_triangles.extend(triangles2)
            colors.extend([layer.color]*len(points))
            
    all_points = numpy.array(all_points)
    all_triangles = numpy.array(all_triangles)
    meshitem = pgo.GLMeshItem(vertexes=all_points, faces=all_triangles, vertexColors=colors,smooth=True)
    return meshitem

def render(rundata,show=False,save_files = False, render_video=True):
    w = ViewWidget()    
    w.setBackgroundColor(1,1,1,1)
        
    meshitems = [gen_mesh_item(body) for body in rundata.rigidbodies]
    [w.addItem(meshitem) for meshitem in meshitems]
    centerpoint = qg.QVector3D(3.5,-1,1)
    
    w.opts['center'] = centerpoint
    w.opts['distance'] = 200
    w.opts['azimuth'] = -45
    w.opts['elevation'] = 45
    w.resize(640,480)
    
    if save_files or render_video:
        if not os.path.exists('render/'):
            os.mkdir('render')
    
    ee = numpy.array(rundata.ee)
    
    w.updateGL()
    
    if show:
        w.show()
    for ii in range(len(ee)):
        for jj,mi in enumerate(meshitems):
            tr = ee[ii,jj]
            tr =qg.QMatrix4x4(*tr.flatten().tolist())
            mi.setTransform(tr)
        w.updateGL()
        if save_files or render_video:
            w.grabFrameBuffer().save('render/img_{0:04d}.png'.format(ii))
        if ii%100==0:
            print(ii)
    if show:
        w.close()

    if render_video:
        if os.path.exists('render.mp4'):
            os.remove('render.mp4')
        subprocess.call('"C:/program files/ffmpeg/bin/ffmpeg" -r {0} -i render/img_%04d.png -vcodec libxvid render.mp4'.format(str(rundata.animation_params.fps)))
    
    if save_files or render_video:
        shutil.rmtree('render')

def update(t,w,ee,meshitems):
    global ii
    if ii<len(ee):
        for jj,mi in enumerate(meshitems):
            tr = ee[ii,jj]
            tr =qg.QMatrix4x4(*tr.flatten().tolist())
            for kk in mi:
                kk.setTransform(tr)
        ii+=1
    else:
        ii=0
        t.stop()
        w.showNormal()

def animate(rundata,thickness):
    w = ViewWidget()    
    w.setBackgroundColor(1,1,1,1)
        
    meshitemss = [body.mesh_items(thickness) for body in rundata.rigidbodies]
    [[w.addItem(meshitem) for meshitem in meshitems] for meshitems in meshitemss]
    centerpoint = qg.QVector3D(3.5,-1,1)
    
    w.opts['center'] = centerpoint
    w.opts['distance'] = 10
    w.opts['azimuth'] = -45
    w.opts['elevation'] = 45
    w.resize(640,480)
    
    ee = numpy.array(rundata.ee)
    
    w.updateGL()
    w.show()
    t = qc.QTimer()
    t.timeout.connect(lambda:update(t,w,ee,meshitemss))
    t.start(rundata.animation_params.t_step*1000)
#    w.close()
    
ii = 0

if __name__=='__main__':
    app = qg.QApplication(sys.argv)
    
    with open('rundata','r') as f:
        rundata = yaml.load(f)
#    render(rundata,show=True,render_video=False)
    animate(rundata)
        
    sys.exit(app.exec_())
