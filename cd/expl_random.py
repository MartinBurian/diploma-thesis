import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator
import random

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
    ax.imshow(m[:,0,:].T, alpha=0.5, interpolation="bilinear")
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
    y=1
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
            
            pos_steps=[]
            
            for d in neigh:
                nextpos=pos+d
                if (nextpos<0).any() or (nextpos>=lim).any() or m[tuple(nextpos)] < 0:
                    continue

                pos_steps.append(d)

            if pos_steps:
                step=random.choice(pos_steps)

            # mark current point as free
            m[tuple(pos)]=-1

            draw()

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

    draw()
