
# coding: utf-8

# In[18]:


from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary
base = BaseOverlay('base.bit')
print('finish')

lib = MicroblazeLibrary(base.RPI, ['uart'])
uart_device = lib.uart_open(14,15)

list2 = [69,71]
while True:
    lib.uart_write(uart_device,list2, len(list2))

print("finish reading")


# In[45]:


from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary
base = BaseOverlay('base.bit')
print('finish')

lib = MicroblazeLibrary(base.RPI, ['uart'])
uart_device = lib.uart_open(14,15)# normal uart speed 9600

def in_range(ascii_in):
    if(ascii_in>=48 and ascii_in<=57):
        return 1
    else:
        return 0
    pass


def read_to_float(list_read):
    result=0
    ten_num=0
    for item in list_read:
        if(item!=46):
            ten_num+=1
        else:
            break
        pass
    
    for s in range(0,len(list_read)-1):
        if(s<=ten_num-1 and in_range(list_read[s])):
            result=result+(list_read[s]-48)*10**(ten_num-s-1)
            pass
        if(s==ten_num):#46,
            pass
        if(s>=ten_num+1 and in_range(list_read[s])):
            result=result+(list_read[s]-48)*10**(ten_num-s)
            pass
    return result
    pass



def get_device_angle(angle_read):
    temp=[0]
    temp1=[0,0,0,0,0,0]
    #angle_read=[0,0,0]

    lib.uart_read(uart_device,temp, 1)
    if(temp[0]==100):#selation d
        lib.uart_read(uart_device,temp1,6)
        temp_fl=read_to_float(temp1)
        if(temp_fl<360 and temp_fl>0):
            angle_read[0]=temp_fl#git rid of unneeded part
        pass
    if(temp[0]==97):#selation a
        lib.uart_read(uart_device,temp1,6)
        temp_fl=read_to_float(temp1)
        if(temp_fl<360 and temp_fl>0):
            angle_read[0]=temp_fl
        #print("a",angle_read[0])
        #print(temp1)
        pass
    if(temp[0]==98):#seaction b
        lib.uart_read(uart_device,temp1,6)
        temp_fl=read_to_float(temp1)
        if(temp_fl<360 and temp_fl>0):
            angle_read[1]=temp_fl
        #print("b",angle_read[1])
        #print(temp1)
        pass
    if(temp[0]==99):#seaction c
        lib.uart_read(uart_device,temp1,6)
        temp_fl=read_to_float(temp1)
        if(temp_fl<360 and temp_fl>0):
            angle_read[2]=temp_fl
        #print("c",angle_read[2])
        #print(temp1)
        pass
    return angle_read

angle_read=[0,0,0]
while True:
    angle_read=get_device_angle(angle_read);
    device_angle_raw=[angle_read[0]-116.1,257.7-angle_read[1],-61.5-(angle_read[1]-angle_read[2])]
    #print(device_angle_raw)
    

print("finish reading")


# In[ ]:


from pynq.overlays.base import BaseOverlay
from pynq.lib import MicroblazeLibrary
base = BaseOverlay('base.bit')
print('finish')

lib = MicroblazeLibrary(base.RPI, ['uart'])
uart_device = lib.uart_open(14,15)# normal uart speed 9600

def in_range(ascii_in):
    if(ascii_in>=48 and ascii_in<=57):
        return 1
    else:
        return 0
    pass


def read_to_float(list_read):
    result=0
    ten_num=0
    for item in list_read:
        if(item!=46):
            ten_num+=1
        else:
            break
        pass
    
    for s in range(0,len(list_read)-1):
        if(s<=ten_num-1 and in_range(list_read[s])):
            result=result+(list_read[s]-48)*10**(ten_num-s-1)
            pass
        if(s==ten_num):#46,
            pass
        if(s>=ten_num+1 and in_range(list_read[s])):
            result=result+(list_read[s]-48)*10**(ten_num-s)
            pass
    return result
    pass


def get_device_angle(angle_read):
    temp=[0]
    temp1=[]
    read_done=[0,0,0]
    #angle_read=[0,0,0]

    lib.uart_read(uart_device,temp, 1)
    if(temp[0]==97):#selation a
        count=0
        while(count<6):
            single=[0]
            lib.uart_read(uart_device,single, 1)
            if(temp!=0):
                temp1.append(single[0])
                count+=1
                pass
            pass
        print(temp1)
        angle_read[0]=read_to_float(temp1)
        temp1=[]
        read_done[0]=1
        pass
    
    lib.uart_read(uart_device,temp, 1)
    if(temp[0]==98):#selation b
        count=0
        while(count<6):
            single=[0]
            lib.uart_read(uart_device,single, 1)
            if(temp!=0):
                temp1.append(single[0])
                count+=1
                pass
            pass
        print(temp1)
        angle_read[1]=read_to_float(temp1)
        temp1=[]
        read_done[1]=1
        pass
    
    lib.uart_read(uart_device,temp, 1)
    if(temp[0]==99):#selation c
        count=0
        while(count<6):
            single=[0]
            lib.uart_read(uart_device,single, 1)
            if(temp!=0):
                temp1.append(single[0])
                count+=1
                pass
            pass
        print(temp1)
        angle_read[2]=read_to_float(temp1)
        temp1=[]
        read_done[2]=1
        pass     

    return angle_read

angle_read=[0,0,0]
while True:
    angle_read=get_device_angle(angle_read);
    device_angle_raw=[angle_read[0]-116.1,257.7-angle_read[1],-61.5-(angle_read[1]-angle_read[2])]
    print(device_angle_raw)
    

print("finish reading")
    

