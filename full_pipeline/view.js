import {C} from '../common/scene.js';
import {interpolate,voxelize} from '../common/math.js';
import {stages} from '../common/catalog.js';
let vox;
export function render(ctx){const {scene:s,data:d,cloud,t,metric,note}=ctx;let stage=Math.min(11,Math.floor(t*12)),u=t===1?1:t*12-stage,av=d.avoidance;
 document.querySelectorAll('.stage').forEach((e,i)=>e.classList.toggle('active',i===stage));metric('Pipeline stage',String(stage+1).padStart(2,'0')+' / 12');metric('Now',stages[stage]);s.environment(d,stage<2?.25:.5);
 if(stage<2){const crop=cloud.filter((p,i)=>p[0]>=0&&p[0]<10&&p[1]>2&&p[1]<12&&p[2]<5&&(stage===0?i%6===0:true));s.points(crop.slice(0,stage===1?Math.floor(crop.length*u):crop.length),'#74d2df',.045);}
 if(stage===2){vox??=voxelize(cloud,.5,d.bounds);s.cubes(vox.slice(0,Math.floor(u*vox.length)),.5,C.occupied,.8);}
 if(stage===3)for(let [a,b] of d.boxes)s.box(a.map(v=>v-.35*u),b.map(v=>v+.35*u),C.inflated,.22);
 if(stage===4)s.points(d.planners['A*'].visited.slice(0,Math.floor(u*d.planners['A*'].visited.length)),C.visited,.12);
 if(stage===5)s.path(d.planners['A*'].path);
 if(stage===6){s.path(d.planners['A*'].path,C.initial);s.path(d.trajectory.pruned);}
 if(stage>=7)s.path(d.trajectory.pruned,C.initial,.022);
 if(stage===7){let blend=35*u**4-84*u**5+70*u**6-20*u**7;s.drone(interpolate([d.start,av.detection],blend));}
 if(stage>=8){s.box(...av.obstacle,C.dynamic,.8);s.box(av.obstacle[0].map(v=>v-.35),av.obstacle[1].map(v=>v+.35),C.inflated,.4,true);}
 if(stage===8)s.drone(av.detection);
 if(stage===9){s.drone(av.detection);s.points(av.local_replan.visited.slice(0,Math.floor(u*av.local_replan.visited.length)),C.visited,.11);if(u>.8)s.path(av.local_detour);}
 if(stage>=10){s.path(av.local_detour);s.drone(stage===11?d.goal:interpolate(av.execution,u));}
 note(stage<3?'Real surface cloud → model-derived occupancy: rack envelopes conservatively close shelving gaps':stage<7?'Same 3D scene · .35 m collision radius · cached A* expansion and validated pruning':stage<10?'Unknown crate intersects the original route · stop and update the map':'Validated straight-segment minimum-snap timing · full stop at corners · cached A* repair reaches the goal');
}
