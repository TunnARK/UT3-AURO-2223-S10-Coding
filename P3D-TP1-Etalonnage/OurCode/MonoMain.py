# -*- coding: utf-8 -*-
from Calibration import MonoCalibration
from Rectification import MonoRectification

# Initialization
MonoCal = MonoCalibration(cols= 4, rows= 11, patternSize_m= 0.03,patternType= 2)
    # patternSize_m: diagonal spacing of 30mm
    # patternType: 2 for asymmetric grid
    
# Acquisition
MonoCal.acquire()

# Calibration
MonoCal.calibrate(framesPath= 'results/acquired')

# Visualization
MonoCal.visualizeBoards()

# Rectification

