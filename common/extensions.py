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


def local_methods(obstacle):
    """Reactive methods share the original start, goal, racks and unknown load.
    Bug/VFH are explicit horizontal-plane adaptations; DWA uses 3D velocities.
    Failures/stalls are reported as outcomes, never replaced with planned routes.
    """
    boxes=np.concatenate((BOXES,[obstacle]));result={};dt=.15
    angles=np.arange(72)*2*np.pi/72;directions=np.c_[np.cos(angles),np.sin(angles),np.zeros(72)]
    for mode in ['Potential field','DWA','VFH','Bug2']:
        p=START.copy();v=np.zeros(3);frames=[];actual=[p.copy()];wall=False;hit_distance=0.;wall_steps=0
        for step in range(220):
            goal=GOAL-p;norm=np.linalg.norm(goal);attraction=goal/max(norm,1)*1.;repulsion=np.zeros(3)
            for lo,hi in boxes:
                delta=p-np.clip(p,lo,hi);dist=np.linalg.norm(delta)
                if .001<dist<1.3:repulsion+=.24*(1/dist-1/1.3)/dist**2*delta/dist
            candidates=[];bad=[];histogram=[];state='Seeking goal'
            if mode=='Potential field':command=attraction+repulsion
            elif mode=='DWA':
                candidates=np.array([np.clip(v+np.array(o)*.22,-1.2,1.2) for o in itertools.product([-1,0,1],repeat=3)])
                # Reachable velocities + rollout + braking-distance admissibility.
                costs=[];rollouts=[]
                for u in candidates:
                    future=p+np.arange(1,9)[:,None]*dt*u;brake=p+u*(1.2+np.linalg.norm(u)/(2*1.5))
                    safe=visible(p,brake,boxes);bad.append(not safe);rollouts.append(future)
                    costs.append(np.linalg.norm(future[-1]-GOAL)+.12/(max(.05,float(clearance(future,boxes).min()))) if safe else 1e6)
                command=candidates[int(np.argmin(costs))] if min(costs)<1e6 else np.zeros(3);candidates=rollouts
            elif mode=='VFH':
                # Binary angular histogram: ray clearance thresholded into blocked sectors.
                for direction in directions:histogram.append(0 if visible(p,p+direction*1.2,boxes) else 1)
                scores=[np.dot(direction,attraction) if not blocked else -100 for direction,blocked in zip(directions,histogram)]
                command=directions[int(np.argmax(scores))]*.8 if max(scores)>-100 else np.zeros(3)
                candidates=[np.array([p,p+direction*1.2]) for direction in directions];bad=[bool(v) for v in histogram];state='Select free angular valley'
            else:
                # Bug2: seek goal along the start-goal line; follow obstacle tangent on contact;
                # leave only at a closer re-intersection with the m-line.
                forward=attraction.copy();forward[2]=0
                if not wall and not visible(p,p+forward*.4,boxes):wall=True;hit_distance=norm;wall_steps=0
                if wall:
                    wall_steps+=1;nearest=min(boxes,key=lambda b:np.linalg.norm(p-np.clip(p,*b)))
                    normal=p-np.clip(p,*nearest);normal[2]=0;dist=np.linalg.norm(normal);normal/=max(dist,1e-9)
                    tangent=np.array([-normal[1],normal[0],0]);command=.65*tangent+normal*(.5-dist)*2
                    axis=GOAL-START;offset=p-START
                    mline=(axis[0]*offset[1]-axis[1]*offset[0])/np.linalg.norm(axis[:2])
                    if abs(mline)<.12 and norm<hit_distance-.4 and wall_steps>12 and visible(p,p+forward*.5,boxes):wall=False
                    state='Follow boundary' if wall else 'Leave at closer m-line crossing'
                else:command=forward
            command=command/max(1,np.linalg.norm(command));nxt=p+dt*command
            if not visible(p,nxt,boxes):command=np.zeros(3);nxt=p.copy();state='Safety stop'
            frames.append(dict(position=p.copy(),prediction=np.array([p,nxt]),rollouts=candidates,bad=bad,attraction=attraction,repulsion=repulsion,control=command,state=state,histogram=histogram))
            p=nxt;v=command;actual.append(p.copy())
            if np.linalg.norm(p-GOAL)<.25:break
        result[mode]=dict(frames=frames,actual=actual,dt=dt,reached=bool(np.linalg.norm(p-GOAL)<.25),final_distance=float(np.linalg.norm(p-GOAL)))
    return result
