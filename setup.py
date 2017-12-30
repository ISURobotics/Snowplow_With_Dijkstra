from distutils.core import setup
import py2exe

setup(
    console = ['posetest.py'],
    options = {
        'py2exe': {
            'packages': ['numpy', 'cv2', 'glob', 'yaml', 'os', 'sys', 'time']
        }
        }
    )
