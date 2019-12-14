from gurobipy import *
import numpy as np
import matplotlib.pyplot as plt
import datetime
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg as LA


#This file is created in the directory from which I execute this file
file = open('all_times.txt','a')

for filename in os.listdir("."):
    if filename.endswith(".lp"): 
        m=read(filename);
        m.optimize();
        print(m.runtime*1000)
        file.write(repr(m.runtime*1000) );
        file.write("\n") 

file.close()