
# coding: utf-8

# In[50]:


import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy.matlib
import cv2
from pynq.overlays.base import BaseOverlay
from pynq.lib.video import *
from pynq.lib import MicroblazeLibrary
base = BaseOverlay("base.bit")

lib = MicroblazeLibrary(base.RPI, ['uart'])
uart_device = lib.uart_open(14,15)# normal uart speed 9600

#####################################################################################

def get_file(file_name,zoom_k):
    point_cloud=[]
    with open(file_name,'r') as file_read:
        print("readin file ", (file_name))
        count=0
        while True:
            line=file_read.readline()
            if not line:
                break
            x_tmp,y_tmp,z_tmp,f_1,f_2,f_3=[float(i) for i in line.split()]
            point = np.zeros((3,1), dtype = np.float) #important for address reflush
            point[0]=x_tmp*zoom_k
            point[1]=y_tmp*zoom_k
            point[2]=z_tmp*zoom_k
            point_cloud.append(point)
            count=count+1
    point_cloud=np.array(point_cloud)
    print(point_cloud)
    print("read finished with ",(len(point_cloud))," points")
    file_read.close()
    return point_cloud

def gen_rotatex(rad_x):
    mat_rotx=np.matlib.identity(4,np.float)
    mat_rotx[1,1]=math.cos(rad_x)
    mat_rotx[1,2]=math.sin(rad_x)
    mat_rotx[2,1]=-1*math.sin(rad_x)
    mat_rotx[2,2]=math.cos(rad_x)
    print(mat_rotx)
    return mat_rotx

def gen_rotatey(rad_y):
    mat_roty=np.matlib.identity(4,np.float)
    mat_roty[0,0]=math.cos(rad_y)
    mat_roty[0,2]=math.sin(rad_y)
    mat_roty[2,0]=-1*math.sin(rad_y)
    mat_roty[2,2]=math.cos(rad_y)
    print(mat_roty)
    return mat_roty


def gen_rotatez(rad_z):
    mat_rotz=np.matlib.identity(4,np.float)
    mat_rotz[0,0]=math.cos(rad_z)
    mat_rotz[0,1]=math.sin(rad_z)
    mat_rotz[1,0]=-1*math.sin(rad_z)
    mat_rotz[1,1]=math.cos(rad_z)
    print(mat_rotz)
    return mat_rotz

def gen_rotmat(rad_x,rad_y,rad_z):
    mat_rotx=gen_rotatex(rad_x)
    mat_roty=gen_rotatey(rad_y)
    mat_rotz=gen_rotatez(rad_z)
    mat_rot=np.matlib.identity(4,np.float)
    mat_rot=mat_rotx*mat_roty*mat_rotz
    return mat_rot

def gen_transmat(x,y,z):
    mat_trans=np.matlib.identity(4,np.float)
    mat_trans[3,0]=x
    mat_trans[3,1]=y
    mat_trans[3,2]=z
    print(mat_trans)
    return mat_trans

def gen_projection( fov_deg, aspect_retio, near_f, far_f):
    fov_rad = 1.0 / math.tan(fov_deg * 0.5 / 180.0 * math.pi)
    result=np.zeros((4,4))
    result[0,0] = aspect_retio * fov_rad
    result[1,1] = fov_rad
    result[2,2] = far_f / (far_f - near_f)
    result[3,2] = (-far_f * near_f) / (far_f - near_f)
    result[2,3] = 1.0
    result[3,3] = 0.0
    print(result)
    return result

def draw_point(p_x,p_y):
    plt.scatter(p_x,p_y)
    plt.xlabel('x_axis')
    plt.ylabel('y_axis')
    plt.title('test_pcl_draw')
    plt.show()

def draw_point_3d(p_x,p_y,p_z):
    fig=plt.figure();
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(p_x,p_y,p_z,c='k',marker='.',s=0.1)
    plt.show()

def draw_point_cv(image_show):
    pass

def point_simplify(point_input,div_rate):
    ori_size=len(point_input)
    point_output=[]
    count=0
    for point_single in point_input:
        if count==0:
            point_output.append(point_single)
        count=count+1
        if count==div_rate:
            count=0
    return point_output

########################################################################

def read_to_float(list_read):
    result=0
    for s in range(0,len(list_read)-1):
        if(s<=2):
            result=result+(list_read[s]-48)*10**(2-s)
            pass
        if(s==3):
            pass
        if(s>=4):
            result=result+(list_read[s]-48)*10**(3-s)
            pass
    return result
    pass


def get_device_angle():
    temp=[0]
    temp1=[0,0,0,0,0,0]
    angle_read=[0,0,0]

    lib.uart_read(uart_device,temp, 1)
    if(temp[0]==97):#selation a
        lib.uart_read(uart_device,temp1,6)
        angle_read[0]=read_to_float(temp1)
        print("a",angle_read[0])
        pass
    if(temp[0]==98):#seaction b
        lib.uart_read(uart_device,temp1,6)
        angle_read[1]=read_to_float(temp1)
        print("b",angle_read[1])
        pass
    if(temp[0]==99):#seaction c
        lib.uart_read(uart_device,temp1,6)
        angle_read[2]=read_to_float(temp1)
        print("c",angle_read[2])
        pass
    return angle_read

########################################################################

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


########################################################################

def virtual_env_setup(x_wall_pos):
    env_cloud=[]
    step=4
    axis_po=0
    for i in range(1,40) :
        point_temp_x = np.zeros((3,1), dtype = np.float) #important for address reflush
        point_temp_y = np.zeros((3,1), dtype = np.float)
        point_temp_z = np.zeros((3,1), dtype = np.float)
        point_temp_x[0]=axis_po
        point_temp_y[1]=axis_po
        point_temp_z[2]=axis_po
        env_cloud.append(point_temp_x)
        env_cloud.append(point_temp_y)
        env_cloud.append(point_temp_z)
        axis_po=axis_po+step
    env_cloud=np.array(env_cloud)
    return env_cloud
    pass

########################################################################


 
def main():
    #point_cloud=get_file("axis.xyz",3)
    print("point cloud file read done")
    #point_cloud=point_simplify(point_cloud,10)
    point_cloud=virtual_env_setup(0.3)
    
    device_angle=[0.2,0,0]
    link_len=[25,100,100]

    #display setup
    near_f=0.1
    far_f=1000.0
    fov_angle=90.0

    screen_w=640
    screen_h=480

    aspect_ratio=screen_h/screen_w

    display_angle=0
    display_offset={0,0,3.0}

    mat_projection=gen_projection(fov_angle, aspect_ratio, near_f, far_f)
    
    #hdmi setup
    Mode = VideoMode(640,480,24)
    hdmi_out = base.video.hdmi_out
    hdmi_out.configure(Mode,PIXEL_BGR)
    hdmi_out.start()
    print("HDMI Initialized\n")

    
    for s in range (0,100):
        display_angle+=0.05
        mat_rot=gen_rotmat(math.pi/4,math.pi/4,math.pi/4)
        mat_trans=gen_transmat(0,0,3.0)
        
        outframe = hdmi_out.newframe()
        outframe[0:480,0:640,:]=255
        
        for point_single in point_cloud:
            point_single4=np.zeros((4,1), dtype = np.float)
            point_single4[3]=1.0

            point_single4[0]=point_single[0]
            point_single4[1]=point_single[1]
            point_single4[2]=point_single[2]

            point_rot=mat_rot*point_single4
            point_trans=mat_trans*point_rot
            point_proj=mat_projection*point_trans

            point_proj[0]+=0.5*screen_w
            point_proj[1]+=0.5*screen_h
            #point_proj[0]*=0.5*screen_w
            #point_proj[1]*=0.5*screen_h
            
            y_int=int(point_proj[1])
            x_int=int(point_proj[0])
            if(x_int>640-1-6):
                x_int=0
            if(x_int<6):
                x_int=0
            if(y_int>480-1-6):
                y_int=0
            if(y_int<6):
                y_int=0;

            #output_image[int(point_proj[1]),int(point_proj[0]),2]=255
            
            outframe[y_int,x_int,0]=0
            outframe[y_int,x_int,1]=0
            #print("point")
            pass
        
        end_pos=forward_kinematics(device_angle,link_len)
        end_pos4=np.zeros((4,1), dtype = np.float)
        for i in range(0,3):
            end_pos4[i]=end_pos[i]
        end_pos4[3]=1.0
        
        pos_rot=mat_rot*end_pos4
        pos_trans=mat_trans*pos_rot
        pos_proj=mat_projection*pos_trans
        
        pos_proj[0]+=0.5*screen_w
        pos_proj[1]+=0.5*screen_h
        
        y_int=int(pos_proj[1])
        x_int=int(pos_proj[0])
        if(x_int>640-1-6):
            x_int=640-1-6
        if(x_int<6):
            x_int=6
        if(y_int>480-1-6):
            y_int=480-1-6
        if(y_int<6):
            y_int=6;
        
        for t1 in range(-3,3):
            for t2 in range(-3,3):
                outframe[y_int+t1,x_int+t2,1]=0
        
        print(end_pos4,"pos")

        print("cycle test")
        hdmi_out.writeframe(outframe)
        print("frame output finished")
        
    pass
    hdmi_out.stop()
    del hdmi_out

 
if __name__ == '__main__':
    main()
    print ('now __name__ is %s' %__name__)


# In[1]:


hdmi_out.stop()
del hdmi_out

