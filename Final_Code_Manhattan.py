# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!


# In[25]:


import math
import sys
import matplotlib.pyplot as plt


# In[26]:


clr = 0.6
d = (0.354/2)+clr
def obstacle(node):
    x = node[0]
    y = node[1]
    c=0
    
    # boundary condition
    if x<-5.55+d or x >5.55-d or y<-5.05+d or y>5.05-d:
        c=1
    # Circle 1
    
    elif ((x+1.6)**2 + (y+4.6)**2 ) < (0.405 + d)**2 :
        c=1
    elif ((x+1.17)**2 + (y+2.31)**2 ) < (0.405 + d)**2 :
        c=1
    elif ((x+1.17)**2 + (y-2.31)**2 ) < (0.405 + d)**2 :
        c=1
    elif ((x+1.65)**2 + (y-4.6)**2 ) < (0.405 + d)**2 :
        c=1
        
    # Circular table
    
    elif ((x+4.05)**2 + (y-3.25)**2 ) < (0.7995 + d)**2 or ((x+2.46)**2 + (y-3.25)**2 ) < (0.7995 + d)**2:
        c=1
    elif -4.05 - d < x and x < -2.45 + d and 2.45 - d < y and y < 4.05 + d:
        c=1
        
    # Long Rectangle touching x-axis (Bottom right)
    
    elif 1.3 - d < x and x < 5.55 + d and -5.05 - d < y and y < -4.7 + d:        # 1
        c=1
    
    elif -0.81 - d < x and x < 1.93 + d and -4.7 - d < y and y < -3.18 + d:        # 2
        c=1

    elif 2.24 - d < x and x < 3.41 + d and -4.7 - d < y and y < -4.12 + d:        # 3
        c=1

    elif 3.72 - d < x and x < 5.55 + d and -4.7 - d < y and y < -3.94 + d:        # 4
        c=1

    # Other Rectangles
    
    elif -1.17 - d < x and x < -0.26 + d and -1.9 - d < y and y < -0.08 + d:        # A
        c=1
        
    elif -0.26 - d < x and x < 1.57 + d and -2.4 - d < y and y < -1.64 + d:        # B
        c=1
        
    elif 2.29 - d < x and x < 3.8 + d and -2.38 - d < y and y < -1.21 + d:        # C
        c=1
        
    elif 4.97 - d < x and x < 5.55 + d and -3.27 - d < y and y < -2.1 + d:        # D
        c=1
        
    elif 4.64 - d < x and x < 5.55 + d and -1.42 - d < y and y < -0.57 + d:        # E
        c=1
        
    elif 4.97 - d < x and x < 5.55 + d and -0.57 - d < y and y < 0.6 + d:        # F
        c=1
        
    elif 1.89 - d < x and x < 5.55 + d and 1.16 - d < y and y < 1.92 + d:        # G
        c=1
    
    elif 2.77 - d < x and x < 3.63 + d and 3.22 - d < y and y < 5.05 + d:        # H
        c=1
        
    elif 4.28 - d < x and x < 4.71 + d and 4.14 - d < y and y < 5.05 + d:        # I
        c=1
    
    return c


# In[27]:


print("Enter initial node parameters")
xi=float(input("x =  "))
yi=float(input("y =  "))
ndi=[xi,yi]
print("Enter goal node parameters")
xg=float(input("x =  "))
yg=float(input("y =  "))
goal=[xg,yg]
theta_i = float (input("Initial robot axis angle with x axis in radians = "))
if (obstacle(goal)==1 or obstacle(ndi)):
    sys.exit("Either goal node or start node lies inside obstacle or outside the workspace")

r = 0.076/2
l = 0.230
clr = 0.6
d = (0.354/2)+clr
rpm1=5
rpm2=10


# In[28]:



# In[29]:


def goal_achieved(node):
    c=0
    if (node[0]-goal[0])**2+(node[1]-goal[1])**2<(0.4)**2:
        c=1
    return c

def movement(node,ul,ur,theta):
    n_nd=[0,0]
    x=node[0]
    y=node[1]
    for i in range(0,400):
        d_theta = (r/l)*(ur-ul)*0.005
        dx = (r/2)*(ul+ur)*(math.cos(theta))*0.005
        dy = (r/2)*(ul+ur)*(math.sin(theta))*0.005
        x = x + dx
        y = y + dy
        theta = theta + d_theta
    n_nd[0]=(n_nd[0]+x)
    n_nd[1]=(n_nd[1]+y)
    n_nd = [ ((math.floor(n_nd[0] * 100)) / 100.0), ((math.floor(n_nd[1] * 100)) / 100.0) ]
    return n_nd,theta


# In[30]:


def distance( node1 , node2 ):
    h = math.sqrt ( (node1[0] - node2[0])**2 +  (node1[1] - node2[1])**2 )
    return h

def checkPoint(node, points):
    flag = 0
    for point in points:
        if (((node[0] - point[0])**2 + (node[1] - point[1])**2) - 0.1**2 < 0):
            return True
    return False


p_nd=[ndi]
c_nd=[ndi]
theta=[theta_i]
cst=[distance(ndi,ndi)]
hst=[distance(goal,ndi)]
act=[[0,0]]
vp_nd=[]
vc_nd=[]
v_theta=[]
v_hst=[]
v_cst=[]
v_act=[]
ndx = ndi
flag=0
i=0
while(flag!=1):

    new_node,new_theta=movement(ndx, 0,rpm1,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.9)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.9),3)
                        hst[cku] = cst[i] + 0.9 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [0,rpm1]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.9),3))
                hst.append(cst[i] + 0.9 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([0,rpm1])
                
    new_node,new_theta=movement(ndx, 0,rpm2,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.8)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.8),3)
                        hst[cku] = cst[i] + 0.8 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [0,rpm2]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.8),3))
                hst.append(cst[i] + 0.8 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([0,rpm2])
                
    new_node,new_theta=movement(ndx, rpm1,rpm2,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.6)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.6),3)
                        hst[cku] = cst[i] + 0.6 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [rpm1,rpm2]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.6),3))
                hst.append(cst[i] + 0.6 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([rpm1,rpm2])
                
    new_node,new_theta=movement(ndx, rpm1,rpm1,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.7)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.7),3)
                        hst[cku] = cst[i] + 0.7 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [rpm1,rpm1]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.7),3))
                hst.append(cst[i] + 0.7 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([rpm1,rpm1])
                
    new_node,new_theta=movement(ndx, rpm2,rpm2,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.5)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.5),3)
                        hst[cku] = cst[i] + 0.5 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [rpm2,rpm2]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.5),3))
                hst.append(cst[i] + 0.5 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([rpm2,rpm2])
                
    new_node,new_theta=movement(ndx, rpm2,rpm1,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.6)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.6),3)
                        hst[cku] = cst[i] + 0.6 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [rpm2,rpm1]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.6),3))
                hst.append(cst[i] + 0.6 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([rpm2,rpm1])
                
    new_node,new_theta=movement(ndx, rpm2,0,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.8)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.8),3)
                        hst[cku] = cst[i] + 0.8 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [rpm2,0]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.8),3))
                hst.append(cst[i] + 0.8 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([rpm2,0]) 
                
    new_node,new_theta=movement(ndx, rpm1,0,theta[i])
    if (obstacle(new_node)!=1):
        if checkPoint(new_node, vc_nd)!= True:
            check=0
            for cku in range(0,len(c_nd)):
                if(new_node == c_nd[cku]):
                    check=1
                    if(cst[cku]>=(cst[i]+0.9)):
                        p_nd[cku]=ndx
                        cst[cku] = round((cst[i]+0.9),3)
                        hst[cku] = cst[i] + 0.9 + distance( goal , new_node )
                        #hst[cku] = distance( goal , new_node )
                        act[cku] = [rpm1,0]
                        theta[cku] = new_theta
                        break
            if check!=1:
                p_nd.append(ndx)
                c_nd.append(new_node)
                theta.append(new_theta)
                cst.append(round((cst[i]+0.9),3))
                hst.append(cst[i] + 0.9 + distance( goal , new_node ))
                #hst.append(distance( goal , new_node ))
                act.append([rpm1,0]) 
                
    vp_nd.append(p_nd.pop(i))
    vc_nd.append(c_nd.pop(i))
    v_theta.append(theta.pop(i))
    v_hst.append(hst.pop(i))
    v_act.append(act.pop(i))
    v_cst.append(cst.pop(i))

    if(flag==0 and c_nd==[]):         
        sys.exit("Path not found")
        
    if(goal_achieved(vc_nd[-1])==1):
        flag=1
        
    if(flag!=1):
        i=hst.index(min(hst))
        ndx=c_nd[i][:]


seq=[v_act[-1]]
x=vp_nd[-1]
i=1
while(x!=ndi):
    if(vc_nd[-i]==x):
        seq.append(v_act[-i])
        x=vp_nd[-i]
    i=i+1     

seqn=[]
seqn.append(vc_nd[-1])
seqn.append(vp_nd[-1])
x=vp_nd[-1]
i=1
while(x!=ndi):
    if(vc_nd[-i]==x):
        seqn.append(vp_nd[-i])
        x=vp_nd[-i]
    i=i+1


for i in vc_nd:
    plt.scatter(i[0],i[1],color='g')
for i in seqn:
    plt.scatter(i[0],i[1],color='r')
plt.show


try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    #res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    #if res==vrep.simx_return_ok:
    #    print ('Number of objects in the scene: ',len(objs))
    #else:
    #    print ('Remote API function call returned with error code: ',res)
    time = 0
#retrieve motor  handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)

    path_speeds= seq[::-1]

    for k in path_speeds:
        time = 0
        err_code1 = 1
        err_code2 = 2
        #print(type(k[0]))
        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            #print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            #print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<2.3):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)

    '''
    steps = [[-4, -4], [-3.85, -4.0], [-3.77, -4.0], [-3.62, -4.0], [-3.54, -4.0]]
    movement = [ [5,10] ]
    #movement =[[5, 10], [10, 5], [5, 10], [10, 5], [5, 10], [10, 5], [0, 10], [10, 0], [0, 10], [10, 0], [5, 10], [5, 5], [10, 5], [5, 10], [5, 0], [5, 10], [10, 5], [10, 10], [5, 10], [10, 5], [10, 10], [0, 5]]
    count=0
    #print(len(movement))
    for i in movement:
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,i[0], vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,i[1],vrep.simx_opmode_streaming)
        if count==len(movement):
            time.sleep(0)
        else :
            #print(steps[count])
            count=count+1
            #print(count)
            time=0
            r,signalValue=vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
            while(time<=2):
                r,signalValue2=vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
                time=signalValue2-signalValue
                print("this",time)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    '''
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

