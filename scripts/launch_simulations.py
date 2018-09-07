#!/usr/bin/env

import os




for i in range(1,3):
	if i==1:
		density=0.1;
	if i==2:
		density=0.2;
	for seed in range(0,10): #0....9
		world_name="density0"+str(i)+"seed"+str(seed)+".world";
		os.system("roslaunch acl_sim sim.launch quad:=SQ01s world_name:="+str(world_name));