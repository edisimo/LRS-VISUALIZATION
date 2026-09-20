"""Small, deterministic teaching algorithms. Coordinates are metres, Z is up."""
from __future__ import annotations
import heapq
import math
import time
import numpy as np
from scipy.linalg import solve

START = np.array([4., 3., 1.5])
GOAL = np.array([5., 9., 1.5])
BOUNDS = np.array([[0., 2., 0.], [10., 12., 5.]])
RADIUS = .35
# Conservative solid envelopes of the four shelving units in fei_lrs_racks/model.sdf.
BOXES = np.array([[[x-1.545,y-.525,0],[x+1.545,y+.525,3.2]]
                  for y in [6.818186,11.3602] for x in [2.695,5.945]])


def clearance(p, boxes=BOXES):
    """Exact signed distance to the union of axis-aligned solid boxes and room boundary."""
    p=np.asarray(p); q=np.abs(p[...,None,:]-boxes.mean(axis=1))-(boxes[:,1]-boxes[:,0])/2
    d=np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(np.max(q,axis=-1),0)
    wall=np.minimum(p-BOUNDS[0],BOUNDS[1]-p).min(axis=-1)
    return np.minimum(d.min(axis=-1),wall)


def visible(a,b,boxes=BOXES,radius=RADIUS):
    """Conservative swept-sphere test via expanded AABBs, exact slab intersection."""
    a=np.asarray(a); b=np.asarray(b)
    if np.any(np.minimum(a,b)<BOUNDS[0]+radius) or np.any(np.maximum(a,b)>BOUNDS[1]-radius):return False
    v=b-a
    for lo,hi in boxes:
        lo=lo-radius;hi=hi+radius;t0=0.;t1=1.
        for k in range(3):
            if abs(v[k])<1e-10:
                if a[k]<lo[k] or a[k]>hi[k]:break
            else:
                t=(lo[k]-a[k])/v[k];u=(hi[k]-a[k])/v[k]
                t0=max(t0,min(t,u));t1=min(t1,max(t,u))
                if t0>t1:break
        else:return False
    return True


def length(p):return float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()) if len(p)>1 else 0.

def path_clearance(p):
    points=np.concatenate([np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.04))) for a,b in zip(p[:-1],p[1:])])
    return float(clearance(points).min())


def graph_search(kind='A*',weight=1.,boxes=BOXES,start=START,goal=GOAL,connectivity=26):
    t=time.perf_counter(); step=.5
    s=tuple(np.rint(start/step).astype(int));g=tuple(np.rint(goal/step).astype(int))
    costs={s:0.};parent={s:s};closed=set();order=[];frontiers=[]
    h=lambda n: float(np.linalg.norm((np.array(n)-g)*step))*(0 if kind=='Dijkstra' else weight)
    queue=[(h(s),0,s)]
    offsets=[(i,j,k) for i in [-1,0,1] for j in [-1,0,1] for k in [-1,0,1] if (i,j,k)!=(0,0,0)]
    # Identical connectivity and swept-sphere edge validation for all graph planners.
    if connectivity==6:offsets=[o for o in offsets if sum(abs(v) for v in o)==1]
    while queue:
        _,cost,n=heapq.heappop(queue)
        if n in closed:continue
        closed.add(n);order.append(np.array(n)*step)
        if n==g:break
        for off in offsets:
            nxt=tuple(n[i]+off[i] for i in range(3));pn=np.array(n)*step;pt=np.array(nxt)*step
            if nxt in closed or not visible(pn,pt,boxes):continue
            ancestor=parent[n] if kind=='Theta*' and visible(np.array(parent[n])*step,pt,boxes) else n
            cand=costs[ancestor]+np.linalg.norm((np.array(ancestor)-nxt)*step)
            if cand<costs.get(nxt,float('inf')):
                costs[nxt]=cand;parent[nxt]=ancestor;heapq.heappush(queue,(cand+h(nxt),cand,nxt))
        # Record actual frontier, not an invented expansion halo.
        if len(order)%20==0:frontiers.append({'at':len(order),'points':[np.array(v)*step for v in set(q[2] for q in queue)-closed]})
    if g not in closed:raise RuntimeError('No grid path')
    path=[g]
    while path[-1]!=s:path.append(parent[path[-1]])
    path=np.array(path[::-1])*step
    return dict(path=path,visited=order,frontiers=frontiers,ms=(time.perf_counter()-t)*1000,nodes=len(order),length=length(path),clearance=path_clearance(path))


def sampling(kind,count=1100):
    t=time.perf_counter();rng=np.random.default_rng(41);points=[START.copy()];parents=[-1];costs=[0.];events=[];best=[];solutions=[]
    def route(i):
        p=[GOAL.copy()]
        while i>=0:p.append(points[i]);i=parents[i]
        return np.array(p[::-1])
    for it in range(count):
        c_best=min((costs[i]+np.linalg.norm(p-GOAL) for i,p in enumerate(points) if i in solutions),default=float('inf'))
        if rng.random()<.10:q=GOAL.copy()
        elif kind=='Informed RRT*' and math.isfinite(c_best):
            # Uniform unit ball mapped into a prolate hyperspheroid aligned with S->G.
            u=rng.normal(size=3);u=u/np.linalg.norm(u)*rng.random()**(1/3)
            axis=(GOAL-START)/np.linalg.norm(GOAL-START);v=np.cross(axis,[0,0,1]);v/=np.linalg.norm(v)
            rot=np.column_stack((axis,v,np.cross(axis,v)));cmin=np.linalg.norm(GOAL-START)
            minor=math.sqrt(max(0,c_best*c_best-cmin*cmin))/2
            q=(START+GOAL)/2+rot@(u*np.array([c_best/2,minor,minor]))
        else:q=rng.uniform(BOUNDS[0]+RADIUS,BOUNDS[1]-RADIUS)
        ps=np.array(points);d=np.linalg.norm(ps-q,axis=1);near=int(d.argmin());delta=q-points[near]
        new=points[near]+delta/max(np.linalg.norm(delta),1e-9)*min(.85,np.linalg.norm(delta))
        if not visible(points[near],new):continue
        neighbors=np.where(np.linalg.norm(ps-new,axis=1)<1.65)[0] if kind!='RRT' else []
        par=near;cost=costs[near]+np.linalg.norm(new-points[near])
        for j in neighbors:
            c=costs[j]+np.linalg.norm(new-points[j])
            if c<cost and visible(points[j],new):par=int(j);cost=c
        idx=len(points);points.append(new);parents.append(par);costs.append(cost);rewires=[]
        for j in neighbors:
            nc=cost+np.linalg.norm(new-points[j])
            if j!=par and nc+1e-8<costs[j] and visible(new,points[j]):
                old=parents[j];diff=nc-costs[j];parents[j]=idx;costs[j]=nc;rewires.append([int(j),old,idx])
                stack=[int(j)]
                while stack:
                    root=stack.pop()
                    for k in range(1,len(parents)):
                        if parents[k]==root:costs[k]+=diff;stack.append(k)
        if np.linalg.norm(new-GOAL)<1.9 and visible(new,GOAL):solutions.append(idx)
        if solutions:
            winner=min(solutions,key=lambda i:costs[i]+np.linalg.norm(points[i]-GOAL));p=route(winner)
            if not best or length(p)<best[-1]['length']-1e-6:best.append(dict(at=idx,path=p,length=length(p)))
        events.append(dict(point=new,parent=par,rewires=rewires,sample=q))
    if not best:raise RuntimeError(f'{kind} seed produced no solution')
    path=best[-1]['path']
    return dict(path=path,events=events,best=best,ms=(time.perf_counter()-t)*1000,nodes=len(points),length=length(path),clearance=path_clearance(path))


def prune(path):
    out=[path[0]];events=[];i=0
    while i<len(path)-1:
        for j in range(len(path)-1,i,-1):
            ok=visible(path[i],path[j]);events.append(dict(a=path[i],b=path[j],ok=ok))
            if ok:out.append(path[j]);i=j;break
    return np.array(out),events


def resample(path,n=45):
    path=np.asarray(path)
    d=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    return np.column_stack([np.interp(np.linspace(0,d[-1],n),d,path[:,k]) for k in range(3)])


def minimum_derivative(path,derivative=4):
    """Degree 2r-1 per segment; fixed waypoint positions; zero endpoint derivatives.
    Joint convex QP minimizes integral ||p^(r)(t)||² with C^(r-1) joins.
    """
    r=derivative;degree=2*r;n=len(path)-1;dur=np.maximum(np.linalg.norm(np.diff(path,axis=0),axis=1)/1.3,1.)
    def basis(t,d):return np.array([0 if k<d else math.factorial(k)/math.factorial(k-d)*t**(k-d) for k in range(degree)])
    Q=np.zeros((degree*n,degree*n));rows=[];rhs=[]
    for s,T in enumerate(dur):
        for i in range(r,degree):
            for j in range(r,degree):Q[s*degree+i,s*degree+j]=math.factorial(i)/math.factorial(i-r)*math.factorial(j)/math.factorial(j-r)*T**(i+j-2*r+1)/(i+j-2*r+1)
        for at,p in [(0,path[s]),(T,path[s+1])]:
            row=np.zeros(degree*n);row[s*degree:(s+1)*degree]=basis(at,0);rows.append(row);rhs.append(p)
    for d in range(1,r):
        for s,at in [(0,0),(n-1,dur[-1])]:
            row=np.zeros(degree*n);row[s*degree:(s+1)*degree]=basis(at,d);rows.append(row);rhs.append(np.zeros(3))
        for s in range(n-1):
            row=np.zeros(degree*n);row[s*degree:(s+1)*degree]=basis(dur[s],d);row[(s+1)*degree:(s+2)*degree]=-basis(0,d);rows.append(row);rhs.append(np.zeros(3))
    A=np.array(rows);B=np.array(rhs);K=np.block([[Q,A.T],[A,np.zeros((len(A),len(A)))]])
    coeff=solve(K,np.vstack((np.zeros((degree*n,3)),B)),assume_a='sym')[:degree*n].reshape(n,degree,3)
    times=[];states=[[] for _ in range(5)];elapsed=0
    for s,T in enumerate(dur):
        for t in np.linspace(0,T,45,endpoint=s==n-1):
            times.append(elapsed+t)
            for d in range(5):states[d].append(basis(t,d)@coeff[s])
        elapsed+=T
    return dict(times=times,position=states[0],velocity=states[1],acceleration=states[2],jerk=states[3],snap=states[4],duration=elapsed,constraint_error=float(np.max(np.abs(A@coeff.reshape(-1,3)-B))),clearance=path_clearance(states[0]))


def optimize(path):
    """CHOMP-inspired covariant descent: smoothness + signed-distance penalty.
    Simplified point objective, not the full arc-length-weighted CHOMP functional.
    """
    p=resample(path);n=len(p);D=np.diff(np.eye(n),n=2,axis=0);A=D.T@D
    metric=np.linalg.inv(A[1:-1,1:-1]+.2*np.eye(n-2));history=[]
    def cost(q):return float(3*np.sum((D@q)**2)+8*np.sum(np.maximum(.85-clearance(q),0)**2))
    for iteration in range(65):
        history.append(dict(path=p.copy(),cost=cost(p),clearance=path_clearance(p)))
        dist=clearance(p);grad=6*A@p
        for k in range(3):
            delta=np.eye(3)[k]*.001;normal=(clearance(p+delta)-clearance(p-delta))/.002
            grad[:,k]-=16*np.maximum(.85-dist,0)*normal
        direction=metric@grad[1:-1];rate=.08
        for _ in range(12):
            q=p.copy();q[1:-1]-=rate*direction
            if cost(q)<cost(p) and all(visible(a,b) for a,b in zip(q[:-1],q[1:])):p=q;break
            rate*=.5
    return history
