# -*- coding: utf-8 -*-
"""
Written by Daniel M. Aukes
Email: danaukes<at>gmail.com
Please see LICENSE for full license.
"""

import subprocess
import os
#cwd = os.getcwd()

if os.path.exists('render.mp4'):
    os.remove('render.mp4')
run1 = subprocess.Popen('"C:/program files/ffmpeg/bin/ffmpeg" -r 30 -i render/%%04d.png -vcodec libxvid render.mp4',stdout = subprocess.PIPE,stderr = subprocess.PIPE)
out1,err1 = run1.communicate()
