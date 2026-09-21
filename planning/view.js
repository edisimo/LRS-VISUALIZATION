import {comparisonNames} from '../common/catalog.js';
import {C} from '../common/scene.js';
const colors=['#d9bbff','#5ee5c0','#f6c572','#85baff','#ffa694','#c2e579','#ee8fce','#4cc9f0','#ffffff'];
export function render(ctx){let {scene:s,data:d,mode,t,metric,note}=ctx;s.environment(d,.55);
 if(mode==='Comparison'){
  const names=comparisonNames.filter(n=>ctx.params.visible?.[n]!==false);names.forEach(n=>s.path(d.planners[n].path,colors[comparisonNames.indexOf(n)],.022));
  metric('Shared collision radius','.35 m');metric('Compared planners',names.length);ctx.table(names.map((n,i)=>{let p=d.planners[n];return {name:n,color:colors[comparisonNames.indexOf(n)],time:Math.round(p.ms)+' ms',nodes:p.nodes,length:p.length.toFixed(2)+' m',clearance:p.clearance.toFixed(2)+' m'};}));note('Cached preparation measurements · interpreted Python · fixed scene and seed · not a universal speed ranking');return;
 }
 let p=d.planners[mode];metric('Prepared compute time',Math.round(p.ms)+' ms');metric('Final length',p.length.toFixed(2)+' m');metric('Minimum clearance',p.clearance.toFixed(2)+' m');
 if(mode==='PRM'){const phase=t*3;s.points(p.points.slice(0,Math.floor(Math.min(1,phase)*p.points.length)),C.frontier,.08);if(phase>1)s.segments(p.edges.slice(0,Math.floor(Math.min(1,phase-1)*p.edges.length)).map(([i,j])=>[p.points[i],p.points[j]]),C.visited,.3);if(phase>2)s.points(p.visited.slice(0,Math.floor((phase-2)*p.visited.length)),C.frontier,.14);if(t>.97)s.path(p.path);metric('Roadmap edges',p.edges.length);note('Seeded collision-free samples · 14 nearest-neighbour candidates · undirected visible edges · Dijkstra on the reusable graph');return;}
 if(p.visited){let n=Math.floor(t*p.visited.length);s.points(p.visited.slice(0,n),C.visited,.095);let front=p.frontiers.filter(f=>f.at<=n).at(-1);if(front&&t<1)s.points(front.points,C.frontier,.12);if(n>0)s.sphere(p.visited[Math.min(n-1,p.visited.length-1)],.14,C.frontier);if(t>.98)s.path(p.path);metric('Expanded so far',n);metric('Graph connectivity','26 neighbours');metric('Grid / collision','.5 m / .35 m');}
 else{
  const n=Math.floor(t*p.events.length);let points=[d.start],parents=[-1],rewires=[];p.events.slice(0,n).forEach(e=>{points.push(e.point);parents.push(e.parent);e.rewires.forEach(([j,old,par])=>{parents[j]=par;});});
  let edges=[];for(let i=1;i<points.length;i++)edges.push([points[i],points[parents[i]]]);s.segments(edges,C.visited,.45);
  p.events.slice(Math.max(0,n-8),n).forEach(e=>e.rewires.forEach(([j,old,par])=>rewires.push([points[j],points[par]])));s.segments(rewires,C.initial,1);
  if(n){let e=p.events[n-1];s.sphere(e.sample,.08,C.frontier);s.line([e.sample,e.point],C.frontier,.5);}
  let best=p.best.filter(b=>b.at<=n).at(-1);if(best){s.path(best.path);metric('Best length now',best.length.toFixed(2)+' m');if(mode==='Informed RRT*')s.ellipsoid(d.start,d.goal,best.length);}
  metric('Tree vertices',points.length);metric('Rewiring events',p.events.slice(0,n).reduce((a,e)=>a+e.rewires.length,0));
 }
 note(mode==='D* Lite'?'D* Lite initial backward search from goal to start · unchanged map · obstacle discovery and incremental repair are shown in Local avoidance':'Deterministic cached history · 3D search · exact segment tests against conservatively inflated rack boxes');
}
