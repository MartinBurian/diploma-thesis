import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator
import random

from mpl_toolkits.mplot3d import Axes3D

W=10
H=10
D=5

lim=np.array([W,D,H])

m=np.zeros(lim)
# m[0,:,:]=1
# m[-1,:,:]=1
# m[:,:,0]=1
# m[:,:,-1]=1
# m[(0,0,-1,-1),:,(0,-1,0,-1)]+=1

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
    plt.yticks(np.arange(5))
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
    #rec# ts=[]
    # for i in range(10):
    #     for j in range(10):
    #         if env[i,y,j]==1:
    #             rects.app
    #end(patches.Rectangle([i-0.5,j-0.5], 1, 1, color='k', alpha=1))
            

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

    im=ax.imshow(m[:,1,:].T, interpolation="nearest", origin="lower", cmap='viridis')

    # for r in rects:
    #     ax.add_patch(r)

    # plot path
    p0=path[path[:,1]==y,:]
    plen=np.sum(np.sqrt(np.sum((p0[1:,:]-p0[:-1,:])**2, axis=1)))
    print("Path length:",plen)
    # p0+=np.random.rand(p0.shape[0], 3)/4 - (1/8)
    ax.plot(p0[:,0], p0[:,2], lw=2, color=ctuorange)
    
    for warp in warps:
        plt.plot(p0[warp-1:warp+1,0], p0[warp-1:warp+1,2], 'k', lw=2)

    plt.colorbar(im)
        
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

path=np.zeros((0,3))
warps=[]
step=None
for y in range(D):
    pos[1]=y
    while True:
        while True:
            print("At",pos)
            path=np.vstack((path, pos))
            if sense(pos):  # position full
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
                if step is not None:
                    pos-=step
                else:
                    break
                continue

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
              

            # draw()
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

            print("warp to",pos)
            warps.append(path.shape[0])
        else:
            break
