"""Deterministic 3D local controllers. No calls to a replacement path planner.

Bug uses sampled obstacle-boundary tangents. All methods track the ORIGINAL
(unrepaired) global polyline while responding to the newly detected obstacle.
They are teaching adaptations, not complete implementations of cited papers.
"""
import itertools
import numpy as np
from common.algorithms import START,GOAL,BOUNDS,BOXES,RADIUS,visible,clearance


def unit(v):
    return v/max(np.linalg.norm(v),1e-9)


def surface_candidates(box,margin=.55):
    """Discretize the six expanded box faces, including edges and face centres."""
    lo,hi=box;lo=lo-margin;hi=hi+margin
    return np.array([lo+(hi-lo)*np.array(t)/2 for t in itertools.product(range(3),repeat=3) if 0 in t or 2 in t])


def tangent_waypoint(position,target,boxes,visited):
    """Local tangent selection on obstacles blocking the target ray.
    Finite boundary samples and visit memory replace unrestricted surface search.
    """
    blockers=[b for b in boxes if not visible(position,target,np.array([b]))]
    candidates=[]
    for box in blockers:
        for q in surface_candidates(box):
            if np.linalg.norm(q-position)<.2 or not visible(position,q,boxes):continue
            if any(np.linalg.norm(q-old)<.3 for old in visited):continue
            score=np.linalg.norm(q-position)+np.linalg.norm(q-target)
            if not visible(q,target,boxes):score+=2.
            candidates.append((score,q))
    if not candidates:return None,[]
    candidates.sort(key=lambda item:item[0]);return candidates[0][1],[q for _,q in candidates[:18]]


def local_methods(obstacle,reference,modes=None):
    boxes=np.concatenate((BOXES,[obstacle]));reference=np.asarray(reference);result={};dt=.15
    azimuth=np.linspace(-np.pi,np.pi,24,endpoint=False);elevation=np.linspace(-np.pi/2,np.pi/2,13)
    directions=np.array([[np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)] for el in elevation for az in azimuth])
    # Nearby ordered samples preserve the supplied route without a replacement planner.
    route=np.concatenate([np.linspace(a,b,int(np.ceil(np.linalg.norm(b-a)/.3))+1)[:-1] for a,b in zip(reference[:-1],reference[1:])] + [reference[-1:]])
    for mode in (modes if modes is not None else ['Bug 3D','Potential field','DWA','VFH 3D']):
        p=START.copy();velocity=np.zeros(3);previous_direction=unit(GOAL-p);actual=[p.copy()];frames=[];guide_index=1
        boundary_target=None;boundary_visited=[];detouring=False
        for step in range(300):
            if mode=='Bug 3D':
                if np.linalg.norm(p-route[guide_index])<.12:
                    guide_index=min(guide_index+1,len(route)-1)
                    detouring=False;boundary_target=None;boundary_visited=[]
                # Skip samples inside the inflated obstruction, retaining the first
                # clear point beyond it as a fixed rejoin target during the detour.
                while guide_index<len(route)-1 and not visible(route[guide_index],route[guide_index],boxes):
                    guide_index+=1
                target=route[guide_index].copy()
                route_target=target.copy()
            else:
                if guide_index<len(reference)-1 and np.linalg.norm(p-reference[guide_index])<.6:guide_index+=1
                target=reference[guide_index].copy()
                if visible(p,GOAL,boxes):target=GOAL.copy()
            attraction=unit(target-p)*min(1.1,np.linalg.norm(target-p));repulsion=np.zeros(3);escape=np.zeros(3)
            rollouts=[];bad=[];histogram=[];chosen_sector=-1;candidate_count=0;state='Track original global route'
            if mode=='Bug 3D':
                if visible(p,target,boxes):
                    boundary_target=None;command=unit(target-p)*min(1.,np.linalg.norm(target-p)/dt)
                    state='Rejoin original global route' if detouring else 'Track original global route'
                else:
                    detouring=True
                    if boundary_target is not None and np.linalg.norm(p-boundary_target)<.18:
                        boundary_visited.append(boundary_target.copy());boundary_target=None
                    if boundary_target is None:
                        boundary_target,options=tangent_waypoint(p,target,boxes,boundary_visited)
                    else:options=[boundary_target]
                    rollouts=[np.array([p,q]) for q in options];bad=[False]*len(rollouts);candidate_count=len(rollouts)
                    if boundary_target is None:command=np.zeros(3);state='No visible boundary sample'
                    else:command=unit(boundary_target-p)*min(1.,np.linalg.norm(boundary_target-p)/dt);state='Follow 3D boundary tangent';target=boundary_target.copy()
            elif mode=='Potential field':
                closest=None;closest_dist=np.inf
                for lo,hi in boxes:
                    delta=p-np.clip(p,lo-RADIUS,hi+RADIUS);dist=np.linalg.norm(delta)+RADIUS
                    if dist<closest_dist:closest_dist=dist;closest=unit(delta)
                    if .001<dist<1.1:repulsion+=.10*(1/dist-1/1.1)/dist**2*unit(delta)
                # Rotational augmentation prevents force cancellation in front of a face.
                # The preferred tangent projects world-up onto that face; on top, use
                # projected target direction. It is a local rule, not a waypoint detour.
                blocked=not visible(p,target,boxes)
                if blocked and closest_dist<1.5:
                    up=np.array([0.,0.,1.]);tangent=up-closest*np.dot(up,closest)
                    if np.linalg.norm(tangent)<.2:tangent=attraction-closest*np.dot(attraction,closest)
                    escape=1.25*unit(tangent);state='Tangential escape + attraction + repulsion'
                # Repel the workspace floor/ceiling as physical limits, not goal guidance.
                for k in range(3):
                    for sign,dist in [(1,p[k]-BOUNDS[0,k]),(-1,BOUNDS[1,k]-p[k])]:
                        if dist<.55:repulsion[k]+=sign*.2*(1/max(.05,dist)-1/.55)
                command=attraction+repulsion+escape
            elif mode=='DWA':
                acceleration=1.6
                offsets=np.array(list(itertools.product(np.linspace(-1,1,5),repeat=3)))
                candidates=velocity+offsets*(acceleration*dt/np.sqrt(3))
                candidates=np.array([u for u in candidates if np.linalg.norm(u)<=1.2+1e-8])
                costs=[];all_rollouts=[];all_bad=[]
                for u in candidates:
                    # Constant-velocity horizon plus an admissible straight braking tail.
                    future=p+np.arange(1,11)[:,None]*dt*u
                    braking_end=future[-1]+unit(u)*np.linalg.norm(u)**2/(2*acceleration)
                    safe=visible(p,braking_end,boxes);all_bad.append(not safe);all_rollouts.append(future)
                    dmin=float(clearance(future,boxes).min())
                    cost=2*np.linalg.norm(future[-1]-target)+.035/max(.03,dmin-RADIUS)+.10*np.linalg.norm(u-velocity)
                    costs.append(cost if safe else 1e8)
                best=int(np.argmin(costs));command=candidates[best] if costs[best]<1e8 else np.zeros(3)
                display=list(range(0,len(candidates),max(1,len(candidates)//28)))[:28]
                if best not in display:display.append(best)
                rollouts=[all_rollouts[k] for k in display];bad=[all_bad[k] for k in display];candidate_count=len(candidates);state='Score reachable 3D velocities + braking'
            else:
                # 3D angular histogram from finite-range, finite-robot collision rays.
                histogram=np.array([not visible(p,p+direction*1.4,boxes,radius=.42) for direction in directions],dtype=int)
                # Discourage one-bin passages using neighbouring angular cells. Polar
                # wrap is periodic in azimuth but not in elevation.
                blocked=histogram.reshape(len(elevation),len(azimuth));padded=blocked.copy()
                padded|=np.roll(blocked,1,axis=1);padded|=np.roll(blocked,-1,axis=1)
                costs=3*(1-directions@unit(target-p))+.6*(1-directions@previous_direction)
                costs[padded.ravel().astype(bool)]=1e8
                chosen_sector=int(np.argmin(costs));command=directions[chosen_sector]*min(.85,np.linalg.norm(target-p)) if costs[chosen_sector]<1e8 else np.zeros(3)
                # All elevation rows and a subset of azimuths are drawn in physical space.
                display=np.arange(0,len(directions),4);rollouts=[np.array([p,p+directions[k]*1.4]) for k in display];bad=[bool(histogram[k]) for k in display];candidate_count=len(directions);histogram=histogram.tolist();state='Select free azimuth AND elevation sector'
            if mode!='DWA':command=command/max(1,np.linalg.norm(command)/1.1)
            nxt=p+dt*command;filtered=False
            # Backtracking only scales the chosen action; no alternate planner intervenes.
            for _ in range(10):
                if visible(p,nxt,boxes):break
                command*=.5;nxt=p+dt*command;filtered=True
            if not visible(p,nxt,boxes):command=np.zeros(3);nxt=p.copy();filtered=True
            frames.append(dict(position=p.copy(),prediction=np.array([p,nxt]),rollouts=rollouts,bad=bad,attraction=attraction,repulsion=repulsion,escape=escape,control=command,state=state,histogram=histogram,chosen_sector=chosen_sector,target=target,candidate_count=candidate_count,filtered=filtered))
            if mode=='Bug 3D':frames[-1].update(route_target=route_target,route_index=guide_index)
            p=nxt;velocity=command;actual.append(p.copy())
            if np.linalg.norm(command)>.05:previous_direction=unit(command)
            if np.linalg.norm(p-GOAL)<.25:break
        result[mode]=dict(frames=frames,actual=actual,dt=dt,reached=bool(np.linalg.norm(p-GOAL)<.25),final_distance=float(np.linalg.norm(p-GOAL)),guidance='Original global path; never repaired',reference=reference,acceleration_limit=1.6 if mode=='DWA' else None)
    return result
