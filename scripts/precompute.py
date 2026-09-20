#!/usr/bin/env python3
"""Explicit preparation; never called by lecture launchers."""
from pathlib import Path
import sys,json,time,hashlib
import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import minimize
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from common.algorithms import *

def dump(path,data):
    def convert(x):
        if isinstance(x,np.ndarray):return x.tolist() if x.dtype.kind=='b' else np.round(x,5).tolist()
        if isinstance(x,np.generic):return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(data,default=convert,separators=(',',':'),allow_nan=False))


def controllers(global_path,detour,extra,H=12):
    result={};dt=.22
    for mode in ['MPC','MPPI']:
        rng=np.random.default_rng(17);pos=START.copy();actual=[pos.copy()];frames=[];u=np.zeros((H,3));t0=time.perf_counter()
        guide_index=1
        for step in range(180):
            # A collision-free detour supplies a terminal guide, avoiding a local minimum.
            if np.linalg.norm(pos-detour[guide_index])<.28 and guide_index<len(detour)-1 and visible(pos,detour[guide_index+1],extra):
                guide_index+=1
            target=detour[guide_index]
            if mode=='MPC':
                ref=resample(detour,150);ix=int(np.linalg.norm(ref-pos,axis=1).argmin())
                for candidate in ref[ix:min(ix+23,len(ref))][::-1]:
                    if visible(pos,candidate,extra):target=candidate;break
            direction=target-pos;nominal=direction/max(np.linalg.norm(direction),1e-9)*1.15
            u=.5*u+.5*nominal
            def rollout(us):return pos+dt*np.cumsum(us,axis=-2)
            def score(us):
                p=rollout(us);d=clearance(p,extra)
                if mode=='MPC':
                    return np.sum((p-target)**2,axis=(-2,-1))*.8+np.sum(np.maximum(.7-d,0)**2,axis=-1)*220+np.sum(us**2,axis=(-2,-1))*.05+np.sum(np.diff(us,axis=-2)**2,axis=(-2,-1))*.12
                return np.sum((p-target)**2,axis=(-2,-1))*3+np.sum(np.maximum(.45-d,0)**2,axis=-1)*500+np.sum(np.maximum(.35-d,0)**2,axis=-1)*20000+np.sum(us**2,axis=(-2,-1))*.05+np.sum(np.diff(us,axis=-2)**2,axis=(-2,-1))*.12
            noise=rng.normal(0,min(.65,max(.08,np.linalg.norm(pos-GOAL)*.3)),(64,H,3));noise=np.cumsum(noise,axis=1)/np.sqrt(np.arange(1,H+1))[None,:,None]
            samples=np.clip(u+noise,-1.5,1.5);samples[0]=np.clip(u,-1.5,1.5);costs=score(samples)
            if mode=='MPC':
                opt=minimize(lambda z:score(z.reshape(H,3)),u.ravel(),method='L-BFGS-B',bounds=[(-1.5,1.5)]*(H*3),options={'maxiter':22,'ftol':1e-5})
                selected=opt.x.reshape(H,3)
            else:
                weights=np.exp(-(costs-costs.min())/3);weights/=weights.sum();selected=np.sum(weights[:,None,None]*samples,axis=0)
            nxt=pos+dt*selected[0]
            # Explicit safety filter; never silently execute an invalid weighted rollout.
            filtered=False
            if not visible(pos,nxt,extra):
                filtered=True;selected=np.zeros_like(selected);nxt=pos.copy()
            prediction=rollout(selected)
            frames.append(dict(position=pos.copy(),rollouts=rollout(samples[:24]),bad=(np.min(clearance(rollout(samples[:24]),extra),axis=1)<RADIUS),prediction=prediction,control=selected[0],filtered=filtered,cost=float(score(selected))))
            pos=nxt;actual.append(pos.copy());u=np.vstack((selected[1:],selected[-1:]))
            if np.linalg.norm(pos-GOAL)<.22:break
        result[mode]=dict(frames=frames,actual=actual,ms=(time.perf_counter()-t0)*1000,dt=dt,horizon=H,samples=64,reached=bool(np.linalg.norm(pos-GOAL)<.25))
        print(mode,'frames',len(frames),'reached',result[mode]['reached'],flush=True)
    return result


def main():
    src=Path(sys.argv[1]) if len(sys.argv)>1 else ROOT.parent/'LRS-URK'
    source=src/'maps/FEI_LRS_PCD/map.pcd'
    cloud=np.loadtxt(source,skiprows=11)
    # Spatial, deterministic downsample, preserving original measured coordinates.
    _,idx=np.unique(np.floor(cloud/.16).astype(int),axis=0,return_index=True)
    reduced=cloud[np.sort(idx)].astype('<f4');reduced.tofile(ROOT/'assets/hangar-points.bin')
    dump(ROOT/'assets/provenance.json',dict(source=str(source.relative_to(src)),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),original_points=len(cloud),display_points=len(reduced),downsample_m=.16,bounds=[cloud.min(0),cloud.max(0)],shelves='models/fei_lrs_racks/model.sdf',world='worlds/fei_lrs_gazebo.world',uav='Simplified procedural quadrotor marker; source mesh inspected, not redistributed.'))
    data=dict(version=1,start=START,goal=GOAL,bounds=BOUNDS,boxes=BOXES,radius=RADIUS,planners={})
    for name in ['RRT','RRT*','Informed RRT*']:
        data['planners'][name]=sampling(name);print(name,data['planners'][name]['nodes'],flush=True)
    raw=graph_search(connectivity=6)['path'];pruned,events=prune(raw)
    # Minimum snap is unconstrained geometrically: collision status is preserved and shown.
    snap=minimum_derivative(pruned);jerk=minimum_derivative(pruned,3)
    spline=BSpline([0,0,0,0,1,1,1,1],pruned,3)(np.linspace(0,1,180))
    shortcut=raw.copy();rng=np.random.default_rng(4);shortcuts=[]
    for _ in range(45):
        if len(shortcut)<3:break
        i,j=sorted(rng.choice(len(shortcut),2,replace=False));ok=visible(shortcut[i],shortcut[j]);a=shortcut[i].copy();b=shortcut[j].copy()
        if ok:shortcut=np.concatenate((shortcut[:i+1],shortcut[j:]))
        shortcuts.append(dict(path=shortcut.copy(),a=a,b=b,ok=ok))
    data['trajectory']=dict(raw=raw,pruned=pruned,pruning=events,shortcuts=shortcuts,spline=spline,spline_clearance=path_clearance(spline),snap=snap,jerk=jerk,chomp=optimize(raw))
    # Unknown suspended load at the first ascent: it intersects the previous planned route.
    center=resample(pruned,21)[7];dynamic=np.array([center-[.65,.65,.65],center+[.65,.65,.65]])
    extra=np.concatenate((BOXES,[dynamic]));repair=graph_search(boxes=extra)
    detour,_events=prune_with_boxes(repair['path'],extra)
    # Keep the validated, generously spaced controller guide independent of comparison connectivity.
    controller_detour,_=prune_with_boxes(graph_search(boxes=extra,connectivity=6)['path'],extra)
    data['avoidance']=dict(obstacle=dynamic,replan=repair,detour=detour,controller_guide=controller_detour,controllers={str(h):controllers(pruned,controller_detour,extra,h) for h in [6,12,18]})
    detection=pruned[0]+.2*(pruned[1]-pruned[0]);repair_local=graph_search(boxes=extra,start=detection)
    repair_local['path']=np.vstack((detection,repair_local['path']))
    repair_local['length']=length(repair_local['path'])
    local_detour,_=prune_with_boxes(repair_local['path'],extra)
    data['avoidance']['detection']=detection
    data['avoidance']['local_replan']=repair_local
    data['avoidance']['local_detour']=local_detour
    # Replanned execution uses validated minimum-snap stop-to-stop segments.
    # Each 7th-degree segment stays on its straight collision-free edge.
    execution=[]
    for a,b in zip(local_detour[:-1],local_detour[1:]):
        t=np.linspace(0,1,55,endpoint=False);s=35*t**4-84*t**5+70*t**6-20*t**7
        execution.extend(a+(b-a)*s[:,None])
    execution.append(local_detour[-1]);data['avoidance']['execution']=execution
    from scripts.refresh_revision import refresh
    data=refresh(data)
    dump(ROOT/'precomputed/lecture.json',data)
    print('Prepared',len(reduced),'points;',round((ROOT/'precomputed/lecture.json').stat().st_size/1e6,2),'MB histories')


def prune_with_boxes(path,boxes):
    out=[path[0]];i=0
    while i<len(path)-1:
        for j in range(len(path)-1,i,-1):
            if visible(path[i],path[j],boxes):out.append(path[j]);i=j;break
    return np.array(out),[]

if __name__=='__main__':main()
