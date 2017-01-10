#!/usr/bin/python

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator
import random
import math

import rospy
from jaco_moveit.srv import *

from itertools import product;

from mpl_toolkits.mplot3d import Axes3D

plt.interactive(True)

W=10
H=9
D=6

lim=np.array([W,D,H])

m=np.zeros(lim)
# m[0,:,:]=1
# m[-1,:,:]=1
# m[:,:,0]=1
# m[:,:,-1]=1
# m[(0,0,-1,-1),:,(0,-1,0,-1)]+=1

d=0.1
OI=-0.45
OJ=0.6
OK=0.1

for i,j,k in product(range(W), range(D), range(H)):
    x = OI+i*d
    y = -(OJ+j*d-0.25)
    z = OK+k*d

    r=math.sqrt(x**2 + y**2 + z**2)

    if r>0.9 or r<0.45:
        m[i,j,k]=-1

env=np.zeros(lim)
env[3,1:3,3]=1
env[4,1:3,2:5]=1
env[5,1:3,0:5]=1
env[6,1:3,0:5]=1
env[7,1:3,2:5]=1
env[4:9,:2,6:8]=1

# env=env[:,1:2,:]

ctublue=[0,110/255,182/255]
ctuorange=[1,115/255,63/255]


def plot_3d():
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d', aspect='equal')

    plt.xticks(np.arange(W))
    plt.yticks(np.arange(D))
    ax.set_xlim(-0.5,9.5)
    ax.set_ylim(-0.5,9.5)
    ax.set_zlim(-0.5,9.5)
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.grid(True, which='minor')

    
    # for tick in ax.axes.get_xticklines():
    #     tick.set_visible(False)

    # for tick in ax.axes.get_yticklines():
    #     tick.set_visible(False)

    ax.plot3D(path[:,0], path[:,1], path[:,2], color=ctuorange, lw=2)
    ax.scatter(*np.where(env==1), s=400, color=ctublue, alpha=1)
    
    for warp in warps:
        ax.plot3D(path[warp-1:warp+1,0], path[warp-1:warp+1,1], path[warp-1:warp+1,2], 'k', lw=2)

    plt.axis('equal')
    ax.autoscale()
    
    plt.tight_layout()

# plot_3d()


def plot_2d_env():
    rects=[]
    for i in range(10):
        for j in range(10):
            if env[i,y,j]==1:
                rects.append(patches.Rectangle([i-0.5,j-0.5], 1, 1, color=ctublue, alpha=1))

    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, aspect='equal')

    plt.xticks(np.arange(W+1))
    plt.yticks(np.arange(H+1))
    plt.axis('scaled')
    ax.set_xlim(-0.5,9.5)
    ax.set_ylim(-0.5,9.5)
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.grid(True, which='minor')

    
    for tick in ax.axes.get_xticklines():
        tick.set_visible(False)

    for tick in ax.axes.get_yticklines():
        tick.set_visible(False)

    for r in rects:
        ax.add_patch(r)

    # plot path
    p0=path[path[:,1]==y,:]
    plen=np.sum(np.sqrt(np.sum((p0[1:,:]-p0[:-1,:])**2, axis=1)))
    print("Path length:",plen)
    p0+=np.random.rand(p0.shape[0], 3)/4 - (1/8)
    ax.plot(p0[:,0], p0[:,2], lw=2, color=ctuorange)
    
    for warp in warps:
        plt.plot(p0[warp-1:warp+1,0], p0[warp-1:warp+1,2], 'k', lw=2)

    plt.tight_layout()
    
def sense(p):
    global env
    return env[tuple(p)]>0


rects=[]

fig = plt.figure()
ax = fig.add_subplot(111)

plt.xticks(np.arange(W+1))
plt.yticks(np.arange(H+1))
plt.axis('equal')
ax.set_xlim(0,10)
ax.set_ylim(0,10)
ax.xaxis.set_minor_locator(AutoMinorLocator(2))
ax.yaxis.set_minor_locator(AutoMinorLocator(2))
ax.grid(True, which='minor')

# plt.show()


def draw():
    ax.clear()
    ax.imshow(m[:,y,:].T, alpha=0.5, interpolation="bilinear", origin="lower")
    ax.grid(True, which='minor')
    for pa in rects:
        ax.add_patch(pa)
    p0=path[path[:,1]==y,:]
    plen=np.sum(np.sqrt(np.sum((p0[1:,:]-p0[:-1,:])**2, axis=1)))
    plt.title(plen)
    p0+=np.random.rand(p0.shape[0], 3)/4
    ax.plot(p0[:,0], p0[:,2])

    for warp in warps:
        plt.plot(p0[warp-1:warp+1,0], p0[warp-1:warp+1,2], 'r')

    plt.axis("equal")
    fig.canvas.draw()

pos=np.zeros((3,), 'i')
pos[0]=0
pos[2]=0

neigh=np.array([[-1,0,0], [1,0,0], [0,0,-1], [0,0,1]], 'i')
nextw=np.zeros(4)

pose_service="/set_exploration_pose"
step_service="/exploration_move"

rospy.wait_for_service(pose_service)
rospy.wait_for_service(step_service)

pose_srv=rospy.ServiceProxy(pose_service, ExplorePoint)
exploration_move=rospy.ServiceProxy(step_service, ExplorePoint)

path=np.zeros((0,3))
warps=[]
step=None
for y in range(D):
    pos[1]=y
    step=None
    while True:
        while True:
            step_success=True

            if m[tuple(pos)]<0:
                break

            print("Moving to ",pos)
            raw_input()

            if step is not None:  # got here by stepping
                # when crosing z=3 and z=4, plan to adjust the orientation
                if (pos[2]==3 and step[2]==1) or (pos[2]==2 and step[2]==-1):
                    step_success=False
                else:
                    print("Stepping")
                    try:
                        res=exploration_move(*step)
                        if not res.reached:
                            step_success=False
                    except rospy.ServiceException, e:
                        print("Stepping failed! Try to plan.")
                        step_success=False

            if step is None or not step_success:
                print("Planning")
                try:
                    res=pose_srv(*pos)
                except rospy.ServiceException, e:
                    print("Planning failed! Exitting.")
                    sys.exit(1)

            if not res.reached:
                print('Destination unreachable! Go somewhere else')
                for d in neigh:
                    nextpos=pos+d
                    if (nextpos<0).any() or (nextpos>=lim).any():
                        continue

                    nextpos=tuple(nextpos)

                    if m[nextpos] >= 0:
                        m[nextpos]+=2

                rects.append(patches.Rectangle(pos[::2]-0.5, 1, 1, color='g', alpha=0.5))
                m[tuple(pos)]=-2
                break

            print("At",pos)
            path=np.vstack((path, pos))
            if res.obstacle:  # position full
                print("full")
                
                for d in neigh:
                    nextpos=pos+d
                    if (nextpos<0).any() or (nextpos>=lim).any():
                        continue

                    nextpos=tuple(nextpos)

                    if m[nextpos] >= 0:
                        m[nextpos]+=2
                        
                rects.append(patches.Rectangle(pos[::2]-0.5, 1, 1, color='g', alpha=0.5))
                m[tuple(pos)]=-2

                if step is not None and step_success:  # I stepped there, I am actually back and need to decide where to go next
                    step=-step
                    pos+=step
                else:
                    break

            # find next direction
            prevstep=step
            step=None
            for i in range(4):
                d=neigh[i,:]
                nextpos=pos+d
                if (nextpos<0).any() or (nextpos>=lim).any():
                    nextw[i]=-1000
                    continue

                nextpos=tuple(nextpos)

                if m[tuple(pos)]>=0 and m[nextpos] >= 0:
                    m[nextpos]+=1

                nextw[i]=m[nextpos]

            cands=np.logical_and(nextw>0, nextw==np.max(nextw))

            # mark current point as free
            m[tuple(pos)]=-1

            # break ties
            if np.sum(cands)>1:
                print("TIE!")
                for i in range(4):
                    nextpos=pos+neigh[i,:]
                    if (nextpos<0).any() or (nextpos>=lim).any() or nextw[i]<0:
                        continue
                    for j in range(4):
                        nnextpos=nextpos+neigh[j,:]
                        if (nnextpos<0).any() or (nnextpos>=lim).any():
                            continue
                        nnextpos=tuple(nnextpos)
                        nextw[i]+=m[nnextpos] if m[nnextpos]>0 else 0
                    print("Step",neigh[i,:],"has potential",nextw[i])
                    
                cands=(nextw==np.max(nextw))

            if np.any(cands):
                if np.sum(cands)>1:
                    print("Choosing at random")
                ni=random.choice(np.where(cands))[0]
                step=neigh[ni,:]
              

            draw()
            # input()

            if step is not None:
                pos+=step
            else:
                break
            
        if (m[:,y,:]>=0).any():
            # missed some in this layer, fix it
            ukno=np.vstack(np.where(m[:,y,:] >= 0))
            mdist=ukno-pos[::2,np.newaxis]
            edist=np.sqrt(np.sum(mdist**2, axis=0))

            closest=np.argmin(edist)
            
            # TODO: how to go
            pos=np.array([ukno[0,closest], y, ukno[1,closest]])
            step=None

            print("warp to",pos)
            warps.append(path.shape[0])
        else:
            break

    draw()
