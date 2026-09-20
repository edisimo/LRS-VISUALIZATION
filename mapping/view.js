import {C} from '../common/scene.js';
import {distance,voxelize,octree} from '../common/math.js';
const cache=new Map();
export function render(ctx){
 const {scene:s,data:d,cloud,mode,t,params:p,metric,note}=ctx;
 const r=+p.resolution||.5;const radius=+p.radius,margin=+p.margin;
 if(mode==='Point cloud'){
  const points=cloud.filter((_,i)=>i%(+p.density||1)===0);s.points(points,'#83cce1',+p.pointSize||.045,points.map(q=>q[2]>3?'#9aaad8':q[2]>1?'#66c8ce':'#586e8b'));
  metric('Surface samples',points.length.toLocaleString());metric('Original PCD','620,311');metric('Units','metres');note('Real hangar PCD · 0.16 m spatial downsample · no inferred free space');return;
 }
 if(['Dense voxels','Sparse voxels','Voxel blocks','Octree'].includes(mode)){
  let key='v'+r;if(!cache.has(key))cache.set(key,voxelize(cloud,r,d.bounds));let vox=cache.get(key),dims=d.bounds[1].map((v,i)=>Math.ceil((v-d.bounds[0][i])/r)),total=dims.reduce((a,b)=>a*b,1);
  if(mode==='Octree'){
   const depth=+p.depth||3;key='oct'+depth;if(!cache.has(key))cache.set(key,octree(cloud,depth));let tree=cache.get(key);
   tree.leaves.forEach(c=>s.box(c.lo,c.lo.map(v=>v+c.size),c.occupied?C.path:C.visited,c.occupied?.35:.09,!c.occupied));
   metric('Leaf nodes',tree.leaves.length.toLocaleString());metric('All nodes',tree.nodes.toLocaleString());metric('Finest cell',(10/2**depth).toFixed(3)+' m');metric('Node estimate',(tree.nodes*40/1024).toFixed(1)+' KiB');note('10 m root cube · split nonempty cells into eight · 40 B/node estimate');return;
  }
  if(mode==='Dense voxels'){
   let empty=[];for(let x=r/2;x<10;x+=r)for(let y=2+r/2;y<12;y+=r)for(let z=r/2;z<5;z+=r)empty.push([x,y,z]);s.points(empty,'#56708a',.025);s.box(...d.bounds,C.visited,.4,true);
  }
  if(mode==='Voxel blocks'){
   const drone=[1+8*t,7+2*Math.sin(t*Math.PI*2),2.5];let blocks=new Map();for(let v of vox){let b=v.map(x=>Math.floor(x/2)*2);if(Math.hypot(b[0]+1-drone[0],b[1]+1-drone[1],b[2]+1-drone[2])<3.6)blocks.set(b.join(),b);}
   let selected=vox.filter(v=>blocks.has(v.map(x=>Math.floor(x/2)*2).join()));s.cubes(selected,r,C.occupied,.7);for(let b of blocks.values())s.box(b,b.map(x=>x+2),C.path,.7,true);s.drone(drone);
   metric('Allocated blocks',blocks.size);metric('Cells per block',Math.round((2/r)**3));metric('Local payload',(blocks.size*(2/r)**3/1024).toFixed(1)+' KiB');note('Rolling 3.6 m region of interest · 2 m blocks · 1 byte per cell');return;
  }
  s.cubes(vox,r,C.occupied,.9);metric('Resolution',r+' m');metric('Grid dimensions',dims.join(' × '));metric('Dense cells',total.toLocaleString());metric('Occupied / allocated',vox.length.toLocaleString());metric('Dense estimate',(total/1024).toFixed(1)+' KiB');metric('Sparse estimate',(vox.length*24/1024).toFixed(1)+' KiB');note('10 × 10 × 5 m crop · dense: 1 B/cell · sparse: 24 B/entry, including assumed index overhead');return;
 }
 s.environment(d,.25);
 let occupied=[],shell=[],free=[],colors=[],z=+p.slice||1.5;
 if(mode==='Occupancy'||mode==='Inflation'){
  for(let x=.25;x<10;x+=.5)for(let y=2.25;y<12;y+=.5)for(let zz=.25;zz<5;zz+=.5){let q=[x,y,zz],dist=distance(q,d.boxes,d.bounds);if(dist<.02)occupied.push(q);else if(dist<(radius+margin)*t)shell.push(q);else if(Math.abs(zz-z)<.3)free.push(q);}
  if(mode==='Occupancy'){
   // Model occupancy is known; no unknown class claimed from a surface-only cloud.
   s.points(cloud.filter(q=>q[0]<10&&q[1]>2),'#62becb',.035);s.cubes(occupied.slice(0,Math.floor(occupied.length*t)),.5,C.occupied,.8);s.cubes(free,.45,C.free,.22);metric('Occupied model cells',occupied.length);metric('Revealed',Math.round(t*100)+'%');note('Surface samples overlaid with known solid rack-envelope occupancy · play to reveal');
  }else{
   s.cubes(occupied,.5,C.occupied,.75);s.cubes(shell,.5,C.inflated,.22);s.path(d.planners['A*'].path,C.path);s.sphere([4.5,8.6,1.5],radius,C.drone,.25);metric('UAV radius',radius.toFixed(2)+' m');metric('Safety margin',margin.toFixed(2)+' m');metric('Inflation',(radius+margin).toFixed(2)+' m');metric('Aisle remaining',Math.max(0,3.492-2*(radius+margin)).toFixed(2)+' m');note('Euclidean distance threshold at voxel centers · nominal cached path uses 0.35 m; slider does not replan');
  }return;
 }
 const points=[];for(let x=.125;x<10;x+=.25)for(let y=2.125;y<12;y+=.25){let q=[x,y,z],dist=distance(q,d.boxes,d.bounds);points.push(q);let value=mode==='Costmap'?Math.exp(-Math.max(0,dist-.35)/(+p.falloff||.7)):Math.max(0,Math.min(1,dist/(+p.range||2)));colors.push(dist<=.35?'#ef936b':mode==='Costmap'?`hsl(${170-140*value},65%,${25+25*value}%)`:`hsl(${25+155*value},60%,${32+15*value}%)`);}
 s.cubes(points,.24,C.free,.72,colors);
 if(mode==='Costmap'){s.path(d.trajectory.pruned,C.initial);s.path(d.trajectory.chomp.at(-1).path,C.path);metric('Falloff length',p.falloff+' m');note('Cost = exp(−max(d−0.35, 0) / falloff) · purple: short pruned route · green: clearance-optimized route');}
 else {let pick=p.pick||[8.5,8,z];pick=[pick[0],pick[1],z];s.sphere(pick,.13,'#fff');metric('Selected clearance',distance(pick,d.boxes,d.bounds).toFixed(2)+' m');metric('Selected x / y',pick.slice(0,2).map(v=>v.toFixed(1)).join(' / '));metric('Color range','0–'+p.range+' m');note('Click slice to probe · signed analytic box distance, including scene boundaries · not a field reconstructed from PCD');}
 metric('Slice altitude',z.toFixed(1)+' m');
}
