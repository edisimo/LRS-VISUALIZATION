"""Additional teaching algorithms; histories contain actual computations."""
import heapq
import itertools
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter1d
from common.algorithms import START,GOAL,BOUNDS,BOXES,RADIUS,visible,length,clearance,path_clearance,resample


def prm(count=350):
    began=time.perf_counter();rng=np.random.default_rng(22);points=[START,GOAL]
    while len(points)<count:
        p=rng.uniform(BOUNDS[0]+RADIUS,BOUNDS[1]-RADIUS)
        if visible(p,p):points.append(p)
    points=np.array(points);tree=cKDTree(points);edges=[];adj=[[] for _ in points]
    for i,p in enumerate(points):
        for j in tree.query(p,k=15)[1][1:]:
            if j>i and visible(p,points[j]):
                cost=float(np.linalg.norm(p-points[j]));adj[i].append((int(j),cost));adj[j].append((i,cost));edges.append([i,int(j)])
    q=[(0.,0)];costs={0:0.};parents={};visited=[];closed=set()
    while q:
        cost,i=heapq.heappop(q)
        if i in closed:continue
        closed.add(i);visited.append(points[i])
        if i==1:break
        for j,c in adj[i]:
            if cost+c<costs.get(j,float('inf')):
                costs[j]=cost+c;parents[j]=i;heapq.heappush(q,(cost+c,j))
    route=[1]
    while route[-1]!=0:route.append(parents[route[-1]])
    path=points[route[::-1]]
    return dict(path=path,points=points,edges=edges,visited=visited,frontiers=[],nodes=count,length=length(path),clearance=path_clearance(path),ms=(time.perf_counter()-began)*1000)


def dstar_lite(obstacle):
    """D* Lite with persistent g/rhs, changed edge costs, moving start and km."""
    began=time.perf_counter();coords=list(itertools.product(range(1,20),range(5,24),range(1,10)))
    points=np.array(coords)*.5;ids={v:i for i,v in enumerate(coords)};adj=[{} for _ in points]
    offsets=[o for o in itertools.product([-1,0,1],repeat=3) if o!=(0,0,0)]
    for i,c in enumerate(coords):
        for off in offsets:
            j=ids.get(tuple(c[k]+off[k] for k in range(3)))
            if j is not None and j>i:
                cost=float(np.linalg.norm(points[i]-points[j])) if visible(points[i],points[j]) else float('inf')
                adj[i][j]=cost;adj[j][i]=cost
    s=ids[tuple((START*2).astype(int))];goal=ids[tuple((GOAL*2).astype(int))];km=0.
    g=np.full(len(points),np.inf);rhs=g.copy();rhs[goal]=0.;queue=[];active={}
    def key(u):
        m=min(g[u],rhs[u]);return (m+np.linalg.norm(points[s]-points[u])+km,m)
    def push(u):
        k=key(u);active[u]=k;heapq.heappush(queue,(*k,u))
    def update(u):
        if u!=goal:rhs[u]=min((c+g[v] for v,c in adj[u].items()),default=np.inf)
        active.pop(u,None)
        if g[u]!=rhs[u]:push(u)
    def peek():
        while queue and active.get(queue[0][2])!=tuple(queue[0][:2]):heapq.heappop(queue)
        return tuple(queue[0][:2]) if queue else (np.inf,np.inf)
    def compute():
        history=[]
        while peek()<key(s) or rhs[s]!=g[s]:
            if not queue:raise RuntimeError('D* Lite disconnected')
            a,b,u=heapq.heappop(queue);active.pop(u,None);old=(a,b);new=key(u)
            if old<new:push(u)
            elif g[u]>rhs[u]:
                g[u]=rhs[u];history.append(points[u])
                for v in adj[u]:update(v)
            else:
                g[u]=np.inf;history.append(points[u]);update(u)
                for v in adj[u]:update(v)
        return history
    def route():
        path=[s]
        while path[-1]!=goal:
            u=path[-1];v=min(adj[u],key=lambda v:adj[u][v]+g[v])
            if v in path or not np.isfinite(adj[u][v]+g[v]):raise RuntimeError('D* Lite route invalid')
            path.append(v)
        return path
    push(goal);initial_visited=compute();initial_ids=route();initial=points[initial_ids];initial_ms=(time.perf_counter()-began)*1000
    old=s;s=initial_ids[1];km+=np.linalg.norm(points[s]-points[old]);extra=np.concatenate((BOXES,[obstacle]));changed=set();changed_edges=0
    for i,neighbors in enumerate(adj):
        for j,cost in list(neighbors.items()):
            if j>i and np.isfinite(cost) and not visible(points[i],points[j],extra):
                adj[i][j]=adj[j][i]=np.inf;changed.update([i,j]);changed_edges+=1
    repair_began=time.perf_counter()
    for u in changed:update(u)
    repaired_visited=compute();repaired=points[route()]
    return dict(path=initial,visited=initial_visited,frontiers=[],nodes=len(initial_visited),length=length(initial),clearance=path_clearance(initial),ms=initial_ms,repair_path=repaired,repair_visited=repaired_visited,repair_ms=(time.perf_counter()-repair_began)*1000,changed=points[sorted(changed)],changed_edges=changed_edges,retained=int(np.isfinite(g).sum()),obstacle=obstacle)


def stomp(path):
    """STOMP-inspired correlated perturbations and cost-weighted feasible updates."""
    rng=np.random.default_rng(71);p=resample(path,40);history=[]
    def cost(q):return float(3*np.sum(np.diff(q,n=2,axis=0)**2)+8*np.sum(np.maximum(.85-clearance(q),0)**2))
    for iteration in range(45):
        noise=gaussian_filter1d(rng.normal(0,.32,(24,len(p),3)),2,axis=1)
        noise[:,0]=0;noise[:,-1]=0;samples=p+noise;costs=np.array([cost(q) for q in samples]);weights=np.exp(-8*(costs-costs.min())/(np.ptp(costs)+1e-9));weights/=weights.sum()
        update=np.sum(weights[:,None,None]*noise,axis=0);chosen=p
        # Preserve feasibility and accept only objective reduction, including weighted updates.
        for q in [p+update,*samples[np.argsort(costs)[:3]]]:
            if cost(q)<cost(chosen) and all(visible(a,b) for a,b in zip(q[:-1],q[1:])):chosen=q
        history.append(dict(path=p.copy(),samples=samples[:8],cost=cost(p),weights=weights[:8],clearance=path_clearance(p)))
        p=chosen
    history.append(dict(path=p,samples=[],cost=cost(p),weights=[],clearance=path_clearance(p)))
    return history



from common.local_control import local_methods
