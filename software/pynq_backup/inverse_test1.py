
# coding: utf-8

# In[12]:


import numpy as np
import math

link_len=[0.05,0.2,0.2]
device_angle=[0.1,0.1,0.1]

def forward_kinematics(device_angle,link_len):
    device_end_pos=[0,0,0]
    
    l1=link_len[0]
    l2=link_len[1]
    l3=link_len[2]
    
    a1=device_angle[0]
    a2=device_angle[1]
    a3=device_angle[2]
    
    x_temp=l2*math.cos(a1)*math.cos(a2) + l3*math.cos(a1)*math.cos(a2)*math.cos(a3) - l3*math.cos(a1)*math.sin(a2)*math.sin(a3)
    y_temp=l2*math.cos(a2)*math.sin(a1) + l3*math.cos(a2)*math.cos(a3)*math.sin(a1) - l3*math.sin(a1)*math.sin(a2)*math.sin(a3)
    z_temp=l1 + l2*math.sin(a2) + l3*math.cos(a2)*math.sin(a3) + l3*math.cos(a3)*math.sin(a2)
    
    device_end_pos[0]=x_temp
    device_end_pos[1]=y_temp
    device_end_pos[2]=z_temp
    
    return device_end_pos
    pass

def inverse_dynamatics(device_angle,link_len,end_force):
    device_torque=[0,0,0]
    
    l1=link_len[0]
    l2=link_len[1]
    l3=link_len[2]
    
    a1=device_angle[0]
    a2=device_angle[1]
    a3=device_angle[2]
    
    fx=end_force[0]
    fy=end_force[1]
    fz=end_force[2]
    
    t1_temp=(fy*math.cos(a1))/(l2*math.cos(a1)**2*math.cos(a2) + l2*math.cos(a2)*math.sin(a1)**2 + l3*math.cos(a1)**2*math.cos(a2)*math.cos(a3) + l3*math.cos(a2)*math.cos(a3)*math.sin(a1)**2 - l3*math.cos(a1)**2*math.sin(a2)*math.sin(a3) - l3*math.sin(a1)**2*math.sin(a2)*math.sin(a3)) - (fx*math.sin(a1))/(l2*math.cos(a1)**2*math.cos(a2) + l2*math.cos(a2)*math.sin(a1)**2 + l3*math.cos(a1)**2*math.cos(a2)*math.cos(a3) + l3*math.cos(a2)*math.cos(a3)*math.sin(a1)**2 - l3*math.cos(a1)**2*math.sin(a2)*math.sin(a3) - l3*math.sin(a1)**2*math.sin(a2)*math.sin(a3))
    t2_temp=(fz*(math.cos(a2)*math.sin(a3) + math.cos(a3)*math.sin(a2)))/(l2*math.cos(a2)**2*math.sin(a3) + l2*math.sin(a2)**2*math.sin(a3)) - (fx*math.cos(a1)*(math.sin(a2)*math.sin(a3) - math.cos(a2)*math.cos(a3)))/(l2*math.sin(a1)**2*math.sin(a2)**2*math.sin(a3) + l2*math.cos(a1)**2*math.cos(a2)**2*math.sin(a3) + l2*math.cos(a1)**2*math.sin(a2)**2*math.sin(a3) + l2*math.cos(a2)**2*math.sin(a1)**2*math.sin(a3)) - (fy*math.sin(a1)*(math.sin(a2)*math.sin(a3) - math.cos(a2)*math.cos(a3)))/(l2*math.sin(a1)**2*math.sin(a2)**2*math.sin(a3) + l2*math.cos(a1)**2*math.cos(a2)**2*math.sin(a3) + l2*math.cos(a1)**2*math.sin(a2)**2*math.sin(a3) + l2*math.cos(a2)**2*math.sin(a1)**2*math.sin(a3))
    t3_temp=-1* (fx*(l2*math.cos(a1)*math.cos(a2) + l3*math.cos(a1)*math.cos(a2)*math.cos(a3) - l3*math.cos(a1)*math.sin(a2)*math.sin(a3)))/(l2*l3*math.cos(a1)**2*math.cos(a2)**2*math.sin(a3) + l2*l3*math.cos(a1)**2*math.sin(a2)**2*math.sin(a3) + l2*l3*math.cos(a2)**2*math.sin(a1)**2*math.sin(a3) + l2*l3*math.sin(a1)**2*math.sin(a2)**2*math.sin(a3)) - (fy*(l2*math.cos(a2)*math.sin(a1) + l3*math.cos(a2)*math.cos(a3)*math.sin(a1) - l3*math.sin(a1)*math.sin(a2)*math.sin(a3)))/(l2*l3*math.cos(a1)**2*math.cos(a2)**2*math.sin(a3) + l2*l3*math.cos(a1)**2*math.sin(a2)**2*math.sin(a3) + l2*l3*math.cos(a2)**2*math.sin(a1)**2*math.sin(a3) + l2*l3*math.sin(a1)**2*math.sin(a2)**2*math.sin(a3)) - (fz*(l2*math.sin(a2) + l3*math.cos(a2)*math.sin(a3) + l3*math.cos(a3)*math.sin(a2)))/(l2*l3*math.cos(a2)**2*math.sin(a3) + l2*l3*math.sin(a2)**2*math.sin(a3))
    
    device_torque[0]=t1_temp
    device_torque[1]=t2_temp
    device_torque[2]=t3_temp
    
    return device_torque
    pass

print(forward_kinematics(device_angle,link_len))
test_force=[-1,-3,-3]
print(inverse_dynamatics(device_angle,link_len,test_force))

