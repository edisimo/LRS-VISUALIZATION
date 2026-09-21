import {C} from '../common/scene.js';
import {interpolate} from '../common/math.js';
export function render(ctx){let {scene:s,data:d,mode,t,params,metric,note}=ctx;const av=d.avoidance;
 if(mode==='D* Lite replanning'){
  const p=d.planners['D* Lite'],stage=Math.min(4,Math.floor(t*5)),detection=p.repair_path[0];
  // The cached moving-start repair begins after the first edge of its own route.
  s.environment(d,.5);s.path(p.path,C.initial,.025);
  if(stage>=1){s.box(...p.obstacle,C.dynamic,.75);s.box(p.obstacle[0].map(v=>v-d.radius),p.obstacle[1].map(v=>v+d.radius),C.inflated,.3,true);}
  if(stage>=2)s.points(p.changed,C.inflated,.08);
  if(stage===2)s.points(p.repair_visited.slice(0,Math.floor((t*5-2)*p.repair_visited.length)),C.frontier,.11);
  if(stage>=3)s.path(p.repair_path,C.path);
  s.drone(stage===0?interpolate(p.path.slice(0,2),t*5):stage<4?detection:interpolate(p.repair_path,(t-.8)*5));
  metric('Map update',['Following global route','New obstacle detected','D* Lite repairing updated map','Replacement route ready','Execute repaired route'][stage]);
  metric('Repair compute',Math.round(p.repair_ms)+' ms');metric('Repair expansions',p.repair_visited.length);metric('Changed edges',p.changed_edges);metric('Finite g after repair',p.retained);
  note('Incremental D* Lite · UAV waits at detection point while persistent g/rhs and moving-start offset km repair the route · orange = changed-edge endpoints; yellow = repair expansions');return;
 }
 if(av.local?.[mode]){
  let c=av.local[mode],i=Math.min(c.frames.length-1,Math.floor(t*c.frames.length)),f=c.frames[i];s.environment(d,.5);s.box(...av.obstacle,C.dynamic,.7);s.path(d.trajectory.pruned,C.initial,.018);s.path(c.actual.slice(0,t===1?c.actual.length:i+1),'white',.025);const current=t===1?c.actual.at(-1):f.position;s.drone(current);
  if(mode==='Potential field'){for(let [v,color] of [[f.attraction,C.path],[f.repulsion,C.dynamic],[f.escape,C.initial],[f.control,C.frontier]]){s.path([f.position,f.position.map((x,k)=>x+v[k])],color,.04);}}
  else {f.rollouts.forEach((q,j)=>s.line([f.position,...q],f.bad[j]?C.dynamic:C.visited,.4));s.path(f.prediction,C.path,.05);}
  if(mode==='Bug 3D')s.sphere(f.route_target,.13,C.initial,.85);
  s.sphere(f.target,.1,C.frontier,.8);
  if(mode==='VFH 3D')ctx.histogram(f.histogram,24,13,f.chosen_sector);
  metric('Control step',i+1+' / '+c.frames.length);metric('State',t===1?(c.reached?'Goal reached':'Goal not reached'):f.state);metric('Goal distance',Math.hypot(...current.map((v,k)=>v-d.goal[k])).toFixed(2)+' m');metric('Altitude',current[2].toFixed(2)+' m');metric('Motion model','XYZ · 3D');metric('Guidance','Original global route');
  if(f.candidate_count)metric('Candidates / sectors',f.candidate_count);
  note(mode==='Potential field'?'Augmented 3D potential field: green attraction, coral repulsion, purple tangential escape, yellow command. Original-route target is yellow; the route is not replanned.':mode==='Bug 3D'?'Follow nearby global-route points in order; detour around an obstruction, then rejoin the first clear point beyond it. Purple = route target; yellow = active tracking or boundary target. This Bug-inspired adaptation has no general completeness guarantee.':mode==='VFH 3D'?'3D histogram: 24 azimuth × 13 elevation bins. Red sectors blocked, green free, white selected. Angular persistence reduces oscillation; no global detour is computed.':'3D DWA: reachable velocity window, 1.5 s rollouts and braking check. Original-route tracking replaces the previous straight-to-goal objective.');return;
 }
 s.environment(d,.5);s.path(d.trajectory.pruned,C.initial,.025);
 if(mode==='A* replanning'){
  let stage=Math.min(4,Math.floor(t*5));if(stage>=1){s.box(...av.obstacle,C.dynamic,.75);s.box(av.obstacle[0].map(v=>v-.35),av.obstacle[1].map(v=>v+.35),C.inflated,.3,true);}
  if(stage===2)s.points(av.local_replan.visited.slice(0,Math.floor((t*5-2)*av.local_replan.visited.length)),C.visited,.09);
  if(stage>=3)s.path(av.local_detour,C.path);
  const pos=stage===0?interpolate([d.start,av.detection],t*5):stage<4?av.detection:interpolate(av.execution,(t-.8)*5);s.drone(pos);metric('Map update',['Following global route','New obstacle detected','A* searching updated map','Replacement route ready','Execute repaired route'][stage]);metric('Repair compute',Math.round(av.local_replan.ms)+' ms');metric('Expanded',av.local_replan.nodes);note('Fresh A* after a map update · not D* Lite · UAV waits at detection point during repair');return;
 }
 s.box(...av.obstacle,C.dynamic,.7);let ctrl=av.controllers[String(params.horizon||12)][mode],i=Math.min(ctrl.frames.length-1,Math.floor(t*ctrl.frames.length)),f=ctrl.frames[i];
 f.rollouts.slice(0,+params.rollouts).forEach((p,j)=>s.line([f.position,...p],f.bad[j]?C.dynamic:C.visited,f.bad[j]?.25:.45));s.path([f.position,...f.prediction],C.path,.04);s.path(ctrl.actual.slice(0,i+2),'#fff',.025);s.path([f.position,ctrl.actual[i+1]],C.frontier,.07);s.drone(f.position);
 metric('Control step',i+1+' / '+ctrl.frames.length);metric('Horizon',(ctrl.horizon*ctrl.dt).toFixed(2)+' s');metric('Action period',ctrl.dt+' s');metric('Candidate samples',mode==='MPPI'?ctrl.samples:'L-BFGS-B solve');metric('Prepared total',Math.round(ctrl.ms)+' ms');metric('Safety filter',f.filtered?'STOP':'Clear first action');ctx.plot(ctrl.frames.map(f=>f.cost),i,'Horizon objective (guide changes each step)',C.path);
 note(mode==='MPC'?'Cached bounded single-integrator MPC · local A* detour supplies a terminal guide · 24 perturbations illustrate alternatives, not optimizer iterates':'Cached simplified MPPI · exponential cost weights over velocity perturbations · detour-guided target · weighted action checked before execution');
}
