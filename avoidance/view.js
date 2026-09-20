import {C} from '../common/scene.js';
import {interpolate} from '../common/math.js';
let potential;
function field(){
 const goal=[5,9,1.5],blocks=[[[3,6,0],[7,6.5,3]],[[3,3,0],[3.5,6,3]],[[6.5,3,0],[7,6,3]]];let p=[5,4,1.5],history=[];
 for(let i=0;i<180;i++){
  const attraction=goal.map((v,k)=>(v-p[k])*.18),repulsion=[0,0,0];
  for(let [lo,hi] of blocks){let nearest=p.map((v,k)=>Math.min(hi[k],Math.max(lo[k],v))),delta=p.map((v,k)=>v-nearest[k]),dist=Math.hypot(...delta);if(dist<2&&dist>.001)delta.forEach((v,k)=>repulsion[k]+=1.2*(1/dist-1/2)/dist**2*v/dist);}
  let force=attraction.map((v,k)=>v+repulsion[k]);history.push({p:[...p],attraction,repulsion,force});let norm=Math.hypot(...force);p=p.map((v,k)=>v+.04*force[k]/Math.max(1,norm));
 }return {history,blocks,goal};
}
export function render(ctx){let {scene:s,data:d,mode,t,params,metric,note}=ctx;const av=d.avoidance;
 if(mode==='Potential field'){
  potential??=field();const {history,blocks,goal}=potential;blocks.forEach(b=>s.box(...b,C.occupied,.7));s.sphere(goal,.2,C.goal);let i=Math.min(history.length-1,Math.floor(t*history.length)),h=history[i];s.path(history.slice(0,i+1).map(h=>h.p));s.drone(h.p);for(let [vector,color] of [[h.attraction,C.path],[h.repulsion,C.dynamic],[h.force,C.frontier]]){s.path([h.p,h.p.map((v,k)=>v+vector[k])],color,.04);s.sphere(h.p.map((v,k)=>v+vector[k]),.07,color);}metric('Net force',Math.hypot(...h.force).toFixed(3));metric('Goal distance',Math.hypot(...goal.map((v,k)=>v-h.p[k])).toFixed(2)+' m');metric('State',i>100?'Stalled at local minimum':'Integrating force');note('Separate U-shaped failure vignette · symmetry cancels lateral escape · forces recomputed from attraction and nearest-surface repulsion');return;
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
