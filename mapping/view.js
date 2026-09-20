import {C} from '../common/scene.js';
import {distance,voxelize,octree} from '../common/math.js';
const cache=new Map();
const memo=(key,fn)=>{if(!cache.has(key))cache.set(key,fn());return cache.get(key);};
const bytes=n=>n>=1048576?(n/1048576).toFixed(2)+' MiB':(n/1024).toFixed(1)+' KiB';
function geometry(s,d,opacity=.2){for(let [a,b] of d.boxes)s.box(a,b,C.occupied,opacity);}
function nearest(p,boxes){let best=null,dist=Infinity;for(let [a,b] of boxes){let q=p.map((v,k)=>Math.max(a[k],Math.min(b[k],v)));let dd=Math.hypot(...q.map((v,k)=>v-p[k]));if(dd<dist){dist=dd;best=q;}}return best;}
function wires(s,cells,color,opacity){const edges=[];for(let c of cells){let corners=Array.from({length:8},(_,i)=>c.lo.map((v,k)=>v+((i>>k)&1)*c.size));for(let i=0;i<8;i++)for(let k=0;k<3;k++)if(!(i&(1<<k)))edges.push([corners[i],corners[i|(1<<k)]]);}s.segments(edges,color,opacity);}
export function render(ctx){
 const {scene:s,data:d,cloud,mode,t,params:p,metric,note}=ctx;const r=+p.resolution||.1,R=+p.radius+(+p.margin),z=+p.slice||1.5;
 if(mode==='Point cloud'){
  const points=cloud.filter((_,i)=>i%(+p.density||1)===0);s.points(points,'#83cce1',+p.pointSize||.045,points.map(q=>q[2]>3?'#9aaad8':q[2]>1?'#66c8ce':'#586e8b'));metric('Surface samples',points.length.toLocaleString());metric('Original PCD','620,311');note('Geometry: measured surface samples. Empty gaps between points do not establish free space.');return;
 }
 if(mode==='Triangle mesh'){
  geometry(s,d,.65);for(let [a,b] of d.boxes){let corners=Array.from({length:8},(_,i)=>a.map((v,k)=>(i>>k)&1?b[k]:v));let edges=[];for(let axis=0;axis<3;axis++)for(let side=0;side<2;side++){let ids=Array.from({length:8},(_,i)=>i).filter(i=>((i>>axis)&1)===side);edges.push([corners[ids[0]],corners[ids[3]]]);}s.segments(edges,C.path,1);s.box(a,b,C.path,.9,true);}metric('Vertices (shared)',32);metric('Triangles',48);metric('Stored information','Surface connectivity');note('Triangle mesh of the SAME simplified rack envelopes, not reconstruction of the PCD. Faces define surfaces; interiors require a closed-mesh convention.');return;
 }
 if(mode==='Elevation map'){
  const cells=memo('height'+r,()=>{let map=new Map();for(let q of cloud){if(q[0]<0||q[0]>=10||q[1]<2||q[1]>=12)continue;let ix=Math.floor(q[0]/r),iy=Math.floor((q[1]-2)/r),key=ix+','+iy;let prev=map.get(key);if(!prev||q[2]>prev[2])map.set(key,[(ix+.5)*r,2+(iy+.5)*r,q[2]]);}return [...map.values()];});s.cubes(cells,r,C.path,.75);metric('XY columns',cells.length.toLocaleString());metric('Payload',bytes(cells.length*4));metric('Heights per column','1');note('2.5D: h(x,y) = highest observed z. Roof and shelves hide the free space below them: compact, but inadequate for arbitrary indoor UAV flight.');return;
 }
 if(['Dense voxels','Sparse voxels','Voxel blocks','Octree'].includes(mode)){
  if(mode==='Octree'){
   let tree=memo('oct'+p.depth,()=>octree(cloud,+p.depth));let groups=new Map();for(let c of tree.leaves){if(c.occupied){if(!groups.has(c.size))groups.set(c.size,[]);groups.get(c.size).push(c.lo.map(v=>v+c.size/2));}}
   for(let [size,ps] of groups)s.cubes(ps,size,C.path,.65);wires(s,tree.leaves.filter(c=>!c.occupied),C.visited,.14);
   metric('Maximum depth',p.depth);metric('Finest width',(10/2**p.depth).toFixed(4)+' m');metric('Leaf cells',tree.leaves.length.toLocaleString());metric('Node storage estimate',bytes(tree.nodes*40));note('Hierarchical storage: split into EIGHT children. Occupied samples use small cells; sample-empty cells stay coarse. 40 B/node estimate, not measured heap.');return;
  }
  let vox=memo('vox'+r,()=>voxelize(cloud,r,d.bounds)),dims=d.bounds[1].map((v,i)=>Math.ceil((v-d.bounds[0][i])/r)),N=dims.reduce((a,b)=>a*b,1),K=vox.length;
  if(mode==='Voxel blocks'){
   const drone=[1+8*t,7+2*Math.sin(t*Math.PI*2),2.5],blocks=new Map();for(let v of vox){let b=v.map(x=>Math.floor(x/2)*2);if(Math.hypot(...b.map((x,k)=>x+1-drone[k]))<3.6)blocks.set(b.join(),b);}
   s.cubes(vox.filter(v=>blocks.has(v.map(x=>Math.floor(x/2)*2).join())),r,C.occupied,.8);for(let b of blocks.values())s.box(b,b.map(x=>x+2),C.path,.7,true);s.drone(drone);metric('Allocated blocks',blocks.size);metric('Cells per 2 m block',Math.round((2/r)**3));metric('Local payload',bytes(blocks.size*(2/r)**3));note('Sparse block directory; dense 1-byte occupancy arrays inside each allocated block. Play to move the local window.');return;
  }
  if(mode==='Dense voxels'){let slice=[];for(let x=r/2;x<10;x+=r)for(let y=2+r/2;y<12;y+=r)slice.push([x,y,.03]);s.points(slice,C.visited,.025);s.box(...d.bounds,C.visited,.4,true);}
  s.cubes(vox,r,C.occupied,.9);let dense=N,sparse=K*24,active=mode==='Dense voxels'?dense:sparse;
  metric('THIS representation',bytes(active));metric('Allocated cells',(mode==='Dense voxels'?N:K).toLocaleString());metric('Dense array · N × 1 B',bytes(dense));metric('Hash map · K × 24 B',bytes(sparse));metric('Cells retained',(100*K/N).toFixed(1)+'%');metric('Sparse / dense',(sparse/dense).toFixed(2)+'×');
  note(`Same ${dims.join(' × ')} grid and same ${K.toLocaleString()} surface cells. Dense stores all ${N.toLocaleString()} cells. Hash stores only K cells, but each costs 1 B value + 23 B estimated indexing overhead. ${sparse<dense?'Sparse saves memory here.':'Hash overhead wins here; sparse is larger.'} Dense allocation markers show one floor slice.`);return;
 }
 if(mode==='Inflation'){
  geometry(s,d,.65);let points=[],base=[],effective;
  if(+p.inflationStorage===1){
   // Refine at the obstacle shell; conservative leaf/cube intersection by its circumradius.
   let leaves=[];function split(lo,size,depth){let center=lo.map(v=>v+size/2),dist=distance(center,d.boxes);if(depth>=+p.depth||Math.abs(dist-R*t)>Math.sqrt(3)*size/2){leaves.push({lo,size,center,dist});return;}for(let i=0;i<8;i++)split(lo.map((v,k)=>v+((i>>k)&1)*size/2),size/2,depth+1);}split([0,2,0],10,0);
   let shell=leaves.filter(c=>c.dist>0&&c.dist<=R*t+Math.sqrt(3)*c.size/2);if(+p.inflationView===1){shell=shell.filter(c=>c.lo[2]<=z&&c.lo[2]+c.size>=z);for(let c of shell)s.box([c.lo[0],c.lo[1],z-.02],[c.lo[0]+c.size,c.lo[1]+c.size,z+.02],C.inflated,.5);}else wires(s,shell,C.inflated,.65);effective=10/2**p.depth;metric('Marked octree leaves',shell.length);metric('Leaf rule','d(center) − half diagonal ≤ R');
  }else{
   const n=Math.ceil((R*t-1e-10)/r);effective=r;let threshold=n*r;
   if(+p.inflationView===1){
    const cells=memo('rack-slice'+r,()=>{let list=[];for(let x=r/2;x<10;x+=r)for(let y=2+r/2;y<12;y+=r)list.push([x,y]);return list;});
    for(let xy of cells){let q=[...xy,z],dist=distance(q,d.boxes);if(dist<=0)base.push(q);else if(dist<=threshold)points.push(q);}
    s.cubes(base,r,C.occupied,1);s.cubes(points,r,C.inflated,.8);metric('Extra forbidden cells',points.length.toLocaleString());
    const front=d.boxes[1][0][1];s.line([[5.95,front,z+.12],[5.95,front-threshold,z+.12]],C.frontier);for(let k=0;k<=n;k++)s.line([[5.87,front-k*r,z+.12],[6.03,front-k*r,z+.12]],C.frontier);
   }else{
    // Cache distance samples once per resolution. Render the exterior shell rather
    // than all hidden interior cubes; the entire enclosed region is forbidden.
    const cells=memo('rack-volume'+r,()=>{let list=[];for(let x=r/2;x<10;x+=r)for(let y=2+r/2;y<12;y+=r)for(let zz=r/2;zz<5;zz+=r){let q=[x,y,zz],dist=distance(q,d.boxes);if(dist>0)list.push({q,dist});}return list;});
    let count=0;for(let c of cells)if(c.dist<=threshold){count++;if(c.dist>Math.max(0,threshold-r*1.05))points.push(c.q);}
    const shellMesh=s.cubes(points,r,C.inflated,.30);if(shellMesh)shellMesh.renderOrder=2;metric('Extra forbidden cells',count.toLocaleString());
   }
   metric('Extra cell layers',n);metric('Rounded grid radius',threshold.toFixed(2)+' m');metric('Rule','ceil((radius + margin) / cell)');
  }
  const drone=[8.8,5,z];s.drone(drone);s.sphere(drone,+p.radius,'#fff',.22);s.sphere(drone,R,C.inflated,.2);metric('Drone diameter',(2*p.radius).toFixed(2)+' m');metric('Radius + margin',R.toFixed(2)+' m');metric('Cell / finest leaf',effective.toFixed(3)+' m');note('Only rack obstacles inflate. No building boundary, no route. Orange = forbidden UAV-center positions. Choose Whole volume or Horizontal slice. The whole-volume exterior shell encloses forbidden space (including hidden interior cells); it is clipped to the teaching volume.');return;
 }
 // One slice and probe shared by all three INFORMATION views; no routes or start/goal markers.
 geometry(s,d,.15);let points=[],colors=[];for(let x=.125;x<10;x+=.25)for(let y=2.125;y<12;y+=.25){let q=[x,y,z],dist=distance(q,d.boxes),value=Math.exp(-Math.max(0,dist)/(+p.falloff));points.push(q);colors.push(mode==='Occupancy'?(dist<=0?'#ff7186':'#3da997'):mode==='Costmap'?(dist<=0?'#ff7186':`hsl(${165-130*value},70%,${30+25*value}%)`):dist<0?'#ff7186':`hsl(${30+150*Math.min(1,dist/p.range)},65%,45%)`);}
 s.cubes(points,.24,C.free,.8,colors);let pick=p.pick||[8.5,8,z];pick=[pick[0],pick[1],z];const dist=distance(pick,d.boxes),q=nearest(pick,d.boxes);s.sphere(pick,.13,'white');
 metric('Probe · x / y',pick.slice(0,2).map(v=>v.toFixed(2)).join(' / '));metric('Slice altitude',z.toFixed(2)+' m');
 if(mode==='Occupancy'){metric('Is this cell occupied?',dist<=0?'YES':'NO');metric('Stored value',dist<=0?'1 · occupied':'0 · free');note('OCCUPANCY answers “Is there an obstacle HERE?” Coral = occupied; green = free. The model supplies known space; no unknown-space inference from missing PCD points. Click to inspect.');}
 else if(mode==='Costmap'){metric('Traversal penalty',(dist<=0?100:100*Math.exp(-dist/p.falloff)).toFixed(0)+' / 100');metric('Falloff',p.falloff+' m');note('COST answers “How undesirable is this cell?” Coral = obstacle / 100; amber = costly nearby free space; green = low cost. C = 100 exp(−max(d,0)/falloff). Click the SAME location across these three views.');}
 else {metric('Distance to rack',dist.toFixed(2)+' m');metric('Stored quantity','Signed metres');if(dist>0){s.path([pick,q],'white',.018);s.sphere(q,.09,C.inflated);}note(`ESDF answers “How far is the nearest surface?” Coral = inside; orange = 0 m; green = ${p.range} m or farther. White ruler joins the probe to its nearest rack surface. Click to measure.`);}
}
