import {renderDstar} from '../planning/view.js';
import {C} from '../common/scene.js';
import {interpolate} from '../common/math.js';
export function render(ctx){let {scene:s,data:d,mode,t,params,metric,note}=ctx;const av=d.avoidance;
 if(mode==='D* Lite replanning'){renderDstar(ctx);return;}
 if(av.local?.[mode]){
  let c=av.local[mode],i=Math.min(c.frames.length-1,Math.floor(t*c.frames.length)),f=c.frames[i];s.environment(d,.5);s.box(...av.obstacle,C.dynamic,.7);s.path(d.trajectory.pruned,C.initial,.018);s.path(c.actual.slice(0,t===1?c.actual.length:i+1),'white',.025);const current=t===1?c.actual.at(-1):f.position;s.drone(current);
  if(mode==='Potential field'){for(let [v,color] of [[f.attraction,C.path],[f.repulsion,C.dynamic],[f.control,C.frontier]]){s.path([f.position,f.position.map((x,k)=>x+v[k])],color,.04);}}
  else {f.rollouts.forEach((q,j)=>s.line([f.position,...q],f.bad[j]?C.dynamic:C.visited,.4));s.path(f.prediction,C.path,.05);}
  if(mode==='VFH')ctx.plot(f.histogram,0,'Blocked angular sectors (0 = open, 1 = blocked)',C.inflated);
  metric('Control step',i+1+' / '+c.frames.length);metric('State',t===1?(c.reached?'Goal reached':'Goal not reached'):f.state);metric('Goal distance',Math.hypot(...current.map((v,k)=>v-d.goal[k])).toFixed(2)+' m');metric('Motion model',['VFH','Bug2'].includes(mode)?'Fixed altitude · 2D':'3D velocity');note(mode==='Potential field'?'Same rack scene; goal attraction + obstacle repulsion. Local minima can stall this method; no hidden global detour or forced failure layout.':mode==='Bug2'?'Bug2-inspired finite-step boundary following in the horizontal plane. It has no general 3D completeness guarantee.':mode==='VFH'?'Binary polar-histogram illustration, not full VFH+. Horizontal sensing misses vertical routes and may oscillate.':'DWA-inspired acceleration-reachable 3D velocity window. Rollouts must allow braking; local minima remain possible.');return;
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
