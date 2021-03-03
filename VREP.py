#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  1 15:01:16 2020

@author: haytam
"""


import pypot
import numpy as np
import pypot.vrep.io as Vrep_io
import time

import math

from membreshaytam import Membres
PI=math.pi

pypot.vrep.close_all_connections()

Poppy = Vrep_io.VrepIO ('127.0.0.1',19997,'/home/haytam/Downloads/poppy_humanoid.ttt')
Poppy.start_simulation()



theta=np.array([PI/2,PI/2,0,0,0])


DH=np.mat([[0,0,theta[0],0],
            [(-PI/2),0,theta[1],0.185],
            [(PI/2),0,theta[2],-0.148],
            [(-PI/2),(-0.01),theta[3],0],
            [(-PI/2),0,theta[4],0.215]])



group=np.array(["l_shoulder_y","l_shoulder_x","l_arm_z","l_elbow_y"])
test = Membres(group,DH,Poppy)



qi=DH[0:4,2]
qf=([[PI/2],[0],[-PI/2],[-PI/2]])
tf=3
vrep_time=0
coeff=test.gen_traj(qi, qf, tf)



while vrep_time<tf:
    
    qd1=test.eval_traj(coeff[0,:], vrep_time)
    Poppy.set_motor_position("l_shoulder_y",qd1)
    qd2=test.eval_traj(coeff[1,:], vrep_time)
    Poppy.set_motor_position("l_shoulder_x",qd2)
    qd3=test.eval_traj(coeff[2,:], vrep_time)
    Poppy.set_motor_position("l_arm_z",qd3)
    qd4=test.eval_traj(coeff[3,:], vrep_time)
    Poppy.set_motor_position("l_elbow_y",qd4)
    vrep_time+=0.05



# Poppy.set_motor_position("l_shoulder_x",0)
# Poppy.set_motor_position("l_arm_z", -PI/2)
# Poppy.set_motor_position("l_elbow_y",-PI/2)
# Poppy.set_motor_position("l_shoulder_y",PI/2)





time.sleep(5)
Poppy.stop_simulation()
Poppy.close()

