# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

import sys
import PyQt5.QtGui as qg
import PyQt5.QtCore as qc
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

def render(rundata,material_properties,size=(1024,768),delete_images=False):
    w = ViewWidget()    
    #w.setBackgroundColor(1,1,1,1)
        
    meshitemss = [body.mesh_items(material_properties) for body in rundata.rigidbodies]
    [[w.addItem(meshitem) for meshitem in meshitems] for meshitems in meshitemss]
    centerpoint = qg.QVector3D(3.5,-1,1)
    
    w.opts['center'] = centerpoint
    w.opts['distance'] = 5
    w.opts['azimuth'] = -45
    w.opts['elevation'] = 30
    w.resize(*size)
    
    if not os.path.exists('render/'):
        os.mkdir('render')
    
    ee = numpy.array(rundata.ee)
    
#    w.updateGL()
    w.paintGL()
    
    w.show()
    for ii in range(len(ee)):
        for jj,mi in enumerate(meshitemss):
            tr = ee[ii,jj]
            tr =qg.QMatrix4x4(*tr.flatten().tolist())
            for kk in mi:
                kk.setTransform(tr)
#        w.updateGL()
        w.paintGL()
        w.grabFrameBuffer().save('render/img_{0:04d}.png'.format(ii))
        if ii%100==0:
            print(ii)
    w.close()
    if os.path.exists('render.mp4'):
        os.remove('render.mp4')
    subprocess.call('ffmpeg -r {0} -i render/img_%04d.png -vcodec libx264 -preset slow -crf 10 render.mp4'.format(str(rundata.animation_params.fps)))
    
    if delete_images:
        shutil.rmtree('render')

import idealab_tools.decorators

@idealab_tools.decorators.static_vars(ii=0)
def update(t,w,ee,meshitems):
    if update.ii<len(ee):
        for jj,mi in enumerate(meshitems):
            tr = ee[update.ii,jj]
            tr =qg.QMatrix4x4(*tr.flatten().tolist())
            for kk in mi:
                kk.setTransform(tr)
        update.ii+=1
    else:
        update.ii=0
        t.stop()
        w.showNormal()

def animate(rundata,material_properties):
    w = ViewWidget()    
    w.setBackgroundColor(1,0,1,1)
        
    meshitemss = [body.mesh_items(material_properties) for body in rundata.rigidbodies]
    [[w.addItem(meshitem) for meshitem in meshitems] for meshitems in meshitemss]
    centerpoint = qg.QVector3D(0,0,0)
    
    w.opts['center'] = centerpoint
    w.opts['distance'] = 5
    w.opts['azimuth'] = -45
    w.opts['elevation'] = 30
    w.resize(1280,1020)
    
    ee = numpy.array(rundata.ee)
    
#    w.updateGL()
    w.paintGL()

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
