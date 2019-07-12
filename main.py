import matplotlib.pyplot as plt
import matplotlib.pyplot as plt1
import matplotlib.pyplot as plt2
import matplotlib.pyplot as plt3
import matplotlib.pyplot as plt4
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import time as ti
import copy
print("Finished Imports at "+str(ti.time())+" seconds.")
st=ti.time()
plt.style.use('dark_background')
fig=plt.figure()
fig.suptitle('Trajectory')
ax=fig.add_subplot(111,projection='3d')
###CURRENT GENERATOR PARAMS
time_elapsed=0
vel_X=0
vel_Y=0
vel_Z=0
windX=0
windY=0
windZ=0

###MAIN GENERATOR PARAMS
period=0.01 ###Make this smaller to be more accurate
slim_period=0.1
override_dens_gen=False ###Do you want to be inaccurate but prove a point? Activate this!
angles_from_thrust=True
drag_degree=2 #Drag is based off of velocity^drag_degree (Normally 2)
g=9.80665
burn_time=140-time_elapsed
coefficient=0.48
coefficient_after_apogee=0.48
frontal_area=0.1641732232
mass=550
s_chute_deployment=200
chute_area=64
chute_coefficient=1.75
goalY=150000

###DEPENDENT PARAMS
    ###IF ANGLES_FROM_THRUST == True:
thrust_X=373.744
thrust_Y=10870.49
thrust_Z=1110

    ###ELSE:
thrust=10941.985415
angle_of_elevation=math.radians(83.35548815)
launch_angle_XZ=math.radians(67.5)

###CALCULATE SOME STUFF
weight=mass*g
if(not angles_from_thrust):
    thrust_Y=math.sin(angle_of_elevation)*thrust
    thrust_XZ=math.cos(angle_of_elevation)*thrust
    thrust_Z=math.sin(launch_angle_XZ)*thrust_XZ
    thrust_X=math.cos(launch_angle_XZ)*thrust_XZ
    print("THRUST X: "+str(thrust_X))
    print("THRUST Y: "+str(thrust_Y))
    print("THRUST Z: "+str(thrust_Z))
    print("THRUST XZ: "+str(thrust_XZ))
if(angles_from_thrust):
    launch_angle_XZ=math.atan(thrust_Z/thrust_X)
    thrust_XZ=math.sqrt((thrust_X**2)+(thrust_Z**2))
    angle_of_elevation=math.atan(thrust_Y/thrust_XZ)
    thrust=math.sqrt((thrust_Y**2)+(thrust_XZ**2))
    print("ANGLE OF ELEVATION: "+str(math.degrees(angle_of_elevation)))
    print("ANGLE X Z: "+str(math.degrees(launch_angle_XZ)))
    print("TOTAL THRUST: "+str(thrust))
print("")
print("")
acc_X=thrust_X/mass
acc_Y=thrust_Y/mass
acc_Z=thrust_Z/mass

###PROGRAM INTERNALS
xfile=open("./x.txt",'w')
yfile=open("./y.txt",'w')
zfile=open("./z.txt",'w')
wxs=[]
wys=[]
wzs=[]
print(wxs)
print(wys)
print(wzs)

periods=0
time=[]
toappend=0
ds=[]
xs=[]
ys=[]
zs=[]
vs=[]
xsn=[]
ysn=[]
zsn=[]
vsn=[]
setc=True
setca=True
setcb=True
def calctrackY(Vi, Fd, C, e):
    Ad=Fd/mass
    a=e-g-Ad
    y=(1/2)*a*(period**2)+Vi*period+C
    Vf=a*period+Vi
    return y,Vf;
def calctrackYn(Vi, C, e):
    a=e-g
    y=(1/2)*a*(period**2)+Vi*period+C
    Vf=a*period+Vi
    return y,Vf;
def calctrackX(Vi, Fd, C, e):
    Ad=Fd/mass
    a=e-Ad
    y=(1/2)*a*(period**2)+Vi*period+C
    Vf=a*period+Vi
    return y,Vf;
def calctrackZ(Vi, Fd, C, e):
    Ad=Fd/mass
    a=e-Ad
    y=(1/2)*a*(period**2)+Vi*period+C
    Vf=a*period+Vi
    return y,Vf;
def tempstop(T):
    if (T<3):
        return 3
    else:
        return T
def pstop(p):
    if isinstance(p, complex):
        print("Enforcing pstop at "+str(p))
        return 0
    else:
        return p
def stoppify(a):
    if a>18000*L:
        return 18000*L
    else:
        return a
def getderiv(a):
    global toappend
    done=[]
    for x in range(0,len(a)-1):
        az=a[x-1]
        aa=a[x]
        ab=a[x+1]
        toappend=abs((ab-aa)/(g*slim_period))
        done.append(toappend)
    done.append(done[len(done)-1])
    return done
def dragforce(v, A, c, h, wind):
    dens=1.421270040489*(0.99985890915519**h)
    vr=v+wind
    if override_dens_gen:
        dens=1.225
    if vr<0:
        return -0.5*A*dens*c*(vr**drag_degree),dens
    else:
        return 0.5*A*dens*c*(vr**drag_degree),dens
def getclosest(a, t):
    lowestV=1000000
    lowestX=0
    pos=0
    for x in a:
        if abs(t-x)<lowestV:
            lowestX=pos
            lowestV=abs(t-x)
        pos+=1
    return lowestX,lowestV
def applystoof(xs, apogee):
    done=[]
    for i in xs:
        done.append(-abs(i-apogee)+apogee)
    return done
def slimdown(a,factor):
    done=[]
    gto=math.floor(len(a)/factor)*factor
    for j in range(0,int(gto),factor):
        done.append(a[j])
    return done
def genzero(le):
    done=[]
    for k in range(0,le):
        done.append(0)
    return done
def trim(a,b,c):
    donea=[]
    doneb=[]
    donec=[]
    for i in range(0,len(b)):
        if b[i]>=0:
            donea.append(a[i])
            doneb.append(b[i])
            donec.append(c[i])
    return donea,doneb,donec
def main(A, c, eX, eY, eZ):
    ViX,ViY,ViZ=vel_X,vel_Y,vel_Z
    x,y,z=0,0,0
    xn,yn,zn=0,0,0

    ViXn=ViX
    ViYn=ViY
    ViZn=ViZ
    while y>=0:
        xs.append(x)
        ys.append(y)
        zs.append(z)
        xsn.append(xn)
        ysn.append(yn)
        zsn.append(zn)
        fdY,densm=dragforce(ViY, A, c, y, -windY)
        fdX,densu=dragforce(ViX, A, c, y, -windX)
        fdZ,densb=dragforce(ViZ, A, c, y, -windZ)
        if(densm!=densu or densm!=densb):
            print("We have big problems.")
        y,ViY=calctrackY(ViY,1*fdY,y,eY)
        x,ViX=calctrackX(ViX,1*fdX,x,eX)
        z,ViZ=calctrackZ(ViZ,1*fdZ,z,eZ)
        yn,ViYn=calctrackYn(ViYn,yn,eY)
        xn,ViXn=calctrackX(ViXn,0*fdX,xn,eX)
        zn,ViZn=calctrackZ(ViZn,0*fdZ,zn,eZ)
        vs.append(math.sqrt(ViX**2+ViY**2+ViZ**2))
        vsn.append(math.sqrt(ViXn**2+ViYn**2+ViZn**2))
        global periods
        global setc
        global setca
        global setcb
        ds.append(densm)
        time.append(periods*period)
        periods+=1
        if(periods*period>s_chute_deployment and setc):
            A=chute_area
            c=chute_coefficient
            setc=False
        if(ViY<0 and setca):
            c=coefficient_after_apogee
            setca=False
        if(periods*period>burn_time and setcb):
            eX,eY,eZ=0,0,0
            setcb=False
            print("Height at Engine Cutoff: "+str(y))
def writeList(a,f):
    for i in a:
        f.write(str(i)+"\n")

main(frontal_area, coefficient, acc_X, acc_Y, acc_Z)
ap=max(xs)
###time, ds, vs, xs, ys, zs
fac=int(period**-1*slim_period)
time=slimdown(time,fac)
ds=slimdown(ds,fac)
vs=slimdown(vs,fac)
xs=slimdown(xs,fac)
ys=slimdown(ys,fac)
zs=slimdown(zs,fac)
vsn=slimdown(vsn,fac)
xsn=slimdown(xsn,fac)
ysn=slimdown(ysn,fac)
zsn=slimdown(zsn,fac)

ysnn=copy.deepcopy(ysn)

xsn,ysn,zsn=trim(xsn,ysn,zsn)

apogee=max(time)/(2*period)
x1=np.linspace(0,ap)
y1=(x1*0)+150000
gs=getderiv(vs)
gsn=getderiv(vsn)
y2=np.linspace(0,max(ysn))
x2=(y2*0)+xs[int(s_chute_deployment/slim_period)]
y3=np.linspace(0,max(ysn))
x3=(y3*0)+xs[int(burn_time/slim_period)]
mu=(max(ds)/goalY*(16/15))**-1
dsa=[i*mu for i in ds]
total_time=periods*period
print("POSITION LIST LENGTH: "+str(len(xs)))
print("TOTAL TIME: "+str(total_time)+" SEC, OR "+str(total_time/60)+" MINUTES.")
print("ALTITUDE AT CHUTE DEPLOYMENT: "+str(ys[int(s_chute_deployment/slim_period)]))
print("VELOCTY AT CHUTE DEPLOYMENT: "+str(vs[int(s_chute_deployment/slim_period)]))
print("MAXIMUM GEE FORCE: "+str(max(gs)))
print("MAXIMUM VELOCITY: "+str(max(vs))+", OR MACH "+str(max(vs)/343))
print("LAST VELOCITY: "+str(vs[int(len(vs)-1)]))
print()
print("Data processing took "+str(ti.time()-st)+" seconds.")
ax.set_xlabel("Downrange X (m)")
ax.set_ylabel("Downrange Z (m)")
ax.set_zlabel("Altitude (m)")
ax.set_xlim(0,max(max(xs),max(zs)))
ax.set_ylim(0,max(max(xs),max(zs)))
ax.set_zlim(0,max(ysn))
plt.plot(xs,zs,ys)
plt.plot(xsn,zsn,ysn)
plt.plot(x1,0*x1,y1)
plt.plot(x2,0*x2+zs[int(s_chute_deployment/slim_period)],y2,alpha=0.5)
plt.plot(x3,0*x3+zs[int(burn_time/slim_period)],y3)
plt.plot(xs,genzero(len(xs)),dsa)
plt.legend(['With Drag','Without Drag','Goal','Chute Deployment','Engine Cutoff','Density (Not to Scale)'])

mu=(max(ds)/max(ys))**-1
dsz=[i*mu for i in ds]
fig1=plt4.figure()
fig1.suptitle('Altitude/Time')
plt4.xlabel("Time")
plt4.ylabel("Altitude (m)")
plt4.xlim(0,max(time))
plt4.ylim(0,max(ys))
plt4.plot(time,ys)
plt4.plot(time,ysnn)
y1_1=np.linspace(0,max(ys))
x1_1=(y1_1*0)+s_chute_deployment
x1_2=(y1_1*0)+burn_time
plt4.plot(x1_1,y1_1)
plt4.plot(x1_2,y1_1)
plt4.plot(time,dsz,alpha=0.5)
plt4.legend(['With Drag','Without Drag','Chute Deployment','Engine Cutoff','Density (Not to Scale)'])

mu=(max(ds)/max(vs))**-1
dsb=[i*mu for i in ds]
fig2=plt1.figure()
fig2.suptitle('Velocity/Time')
plt1.xlabel("Time")
plt1.ylabel("Velocity (m/sec)")
plt1.xlim(0,max(time))
plt1.ylim(0,max(vs))
plt1.plot(time, vs)
plt1.plot(time, vsn)
y1_1=np.linspace(0,max(vs))
x1_1=(y1_1*0)+s_chute_deployment
x1_2=(y1_1*0)+burn_time
plt1.plot(x1_1,y1_1)
plt1.plot(x1_2,y1_1)
plt1.plot(time,dsb,alpha=0.5)
plt1.legend(['With Drag','Without Drag','Chute Deployment','Engine Cutoff','Density (Not to Scale)'])

mu=(max(ds)/max(gs))**-1
dsc=[i*mu for i in ds]
fig3=plt2.figure()
fig3.suptitle('Gravity/Time')
y2_1=np.linspace(0,max(gs))
x2_1=(y1_1*0)+s_chute_deployment
x2_2=(y1_1*0)+burn_time
plt2.xlabel('Time')
plt2.ylabel('Gravity (Gees)')
plt2.xlim(0,max(time))
plt2.ylim(0,max(gs))
plt2.plot(time,gs)
plt2.plot(time,gsn)
plt2.plot(x2_1,y2_1,alpha=0.5)
plt2.plot(x2_2,y2_1)
plt2.plot(time,dsc,alpha=0.5)
plt2.legend(['With Drag','Without Drag','Chute Deployment','Engine Cutoff','Density (Not to Scale)'])

print("")
print('DOWNRANGE STATS:')
print("DOWNRANGE X: "+str(max(xs)))
print("DOWNRANGE Z: "+str(max(zs)))
tot=math.sqrt(max(xs)**2+max(zs)**2)
print("TOTAL DOWNRANGE: "+str(tot))

print("")
if(max(ys)>=goalY):
    print("VERTICAL GOAL FULFILLED! OVERSHOT BY "+str(max(ys)-goalY)+" METERS.")
else:
    print("VERTICAL GOAL NOT FULFILLED. ALTITUDE WAS "+str(max(ys))+" METERS, "+str(goalY-max(ys))+" METERS SHORT.")
fig.show()
plt.show()
