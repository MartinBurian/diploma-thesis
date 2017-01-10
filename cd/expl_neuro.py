import numpy as np
from scipy.signal import convolve2d
from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator

from mpl_toolkits.mplot3d import Axes3D

W=10
H=10
D=5

lim=np.array([W,D,H])

m=np.zeros(lim)

env=np.zeros(lim)
env[3,1:3,3 ]=1
env[4,1:3,2:5]=1
env[5,1:3,0:5]=1
env[6,1:3,0:5]=1
env[7,1:3,2:5]=1
env[4:9,:2,6:8]=1

def sense(p):
    global env
    return env[tuple(p)]>0

sq2=np.sqrt(2)/2
# dkernel=np.array([[.7,.7/sq2,.7],[.7/sq2, 0, .7/sq2],[.7, .7/sq2, .7]])
dkernel=np.array([[0,.7/sq2,0],[.7/sq2, 0, .7/sq2],[0, .7/sq2, 0]])


def dact(act, ext):
    conv=convolve2d(np.maximum(act,0), dkernel, 'same') / convolve2d(np.ones_like(activity), dkernel,'same')
    neg_distr=convolve2d(np.minimum(act,0), dkernel, 'same')
    
    return -A*act \
        +(B-act)*(conv + np.maximum(ext_input,0)) \
        -(D+act)*(np.maximum(-ext_input,0))


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
    # rects=[]
    # for i in range(10):
    #     for j in range(10):
    #         if env[i,y,j]==1:
    #             rects.append(patches.Rectangle([i-0.5,j-0.5], 1, 1, color=ctublue, alpha=1))

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

    # for r in rects:
    #     ax.add_patch(r)

    # plot path
    im=ax.imshow(activity.T, interpolation="nearest", cmap="viridis")
    
    p0=path[path[:,1]==y,:]
    plen=np.sum(np.sqrt(np.sum((p0[1:,:]-p0[:-1,:])**2, axis=1)))
    print("Path length:",plen)
    # p0+=np.random.rand(p0.shape[0], 3)/4 - (1/8)
    ax.plot(p0[:,0], p0[:,2], lw=2, color=ctuorange)

    for warp in warps:
        plt.plot(p0[warp-1:warp+1,0], p0[warp-1:warp+1,2], 'k', lw=2)

    plt.colorbar(im)
    
    plt.tight_layout()




A=50
B=2
D=1
E=100
C=0.00

STEP=0.005


pos=np.zeros((3,), 'i')
neigh=np.array([[-1,0,0], [1,0,0], [0,0,-1], [0,0,1]], 'i')

path=np.zeros((0,3))

turn_values=[.5, 0, 1]

rects=[]

plt.interactive(True)

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
    ax.imshow(activity.T, interpolation="none", cmap="viridis")
    ax.grid(True, which='minor')
    for pa in rects:
        ax.add_patch(pa)
    p0=path[path[:,1]==y,:]
    plen=np.sum(np.sqrt(np.sum((p0[1:,:]-p0[:-1,:])**2, axis=1)))
    plt.title(plen)
    p0+=np.random.rand(p0.shape[0], 3)/4-0.25
    ax.plot(p0[:,0], p0[:,2])
    
    for warp in warps:
        plt.plot(p0[warp-1:warp+1,0], p0[warp-1:warp+1,2], 'r')

    plt.axis("equal")
    fig.canvas.draw()

warps=[]
# explore
for y in range(5):
    pos[1]=y
    activity=np.zeros((W,H))
    ext_input=np.ones((W,H))*E

    # a few steps to stabilize the initial state
    for i in range(10):
        k1=dact(activity, ext_input)
        k2=dact(activity + (STEP/2)*k1, ext_input)
        k3=dact(activity + (STEP/2)*k2, ext_input)
        k4=dact(activity+STEP*k3, ext_input)
        
        activity+=STEP/6 * (k1 + 2*k2 + 2*k3 + k4)
        print(activity)

    step=None
    
    while True:
        while True:
            print("At",pos)
            path=np.vstack((path, pos))
            
            if sense(pos):  # position full
                print("full")
                m[tuple(pos)]=-2
                ext_input[tuple(pos[::2])]=-E
                
                rects.append(patches.Rectangle(pos[::2]-0.5, 1, 1, color='g', alpha=0.5))
                if step is not None:
                    pos-=step
                else:
                    break
                continue

            m[tuple(pos)]=-1
            ext_input[tuple(pos[::2])]=0

            for i in range(5):
                k1=dact(activity, ext_input)
                k2=dact(activity + (STEP/2)*k1, ext_input)
                k3=dact(activity + (STEP/2)*k2, ext_input)
                k4=dact(activity + STEP*k3, ext_input)

                activity+=STEP/6 * (k1 + 2*k2 + 2*k3 + k4)

            draw()
            # input()
            
            maxd=activity[tuple(pos[::2])]
            prev_step=step
            step=None
            for d in neigh:
                nextpos=pos+d
                if (nextpos<0).any() or (nextpos>=lim).any():
                    continue

                nextpos=tuple(nextpos[::2])
                dir_contrib = turn_values[np.sum(d==prev_step)-1] if prev_step is not None else 0
                next_val=activity[nextpos] + C*dir_contrib

                if next_val>maxd:
                    maxd=next_val
                    step=d

            if step is not None:
                pos+=step
            else:
                break
            
        if (m[:,y,:]>=0).any():
            print("deadlock, just wait",pos)
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
