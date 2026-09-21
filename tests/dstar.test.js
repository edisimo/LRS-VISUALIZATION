import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {render as planning} from '../planning/view.js';
import {render as avoidance} from '../avoidance/view.js';
const data=JSON.parse(readFileSync(new URL('../precomputed/lecture.json',import.meta.url)));
const p=data.planners['D* Lite'];
function frame(render,mode,t){
 const calls={};const metrics={};
 const scene=new Proxy({}, {get:(_,name)=>(...args)=>(calls[name]??=[]).push(args)});
 render({scene,data,mode,t,params:{},metric:(k,v)=>metrics[k]=v,note:()=>{}});
 return {calls,metrics};
}
test('planning D* Lite animates only initial search and reveals its initial route',()=>{
 for(const t of [0,.1,.3,.5,.7,.9,1]){
  const {calls,metrics}=frame(planning,'D* Lite',t);
  assert.equal(calls.box,undefined);assert.equal(calls.drone,undefined);
  assert.equal(metrics['Expanded so far'],Math.floor(t*p.visited.length));
  assert.deepEqual(calls.points[0][0],p.visited.slice(0,Math.floor(t*p.visited.length)));
  if(t<.98)assert.equal(calls.path,undefined);
  else assert.deepEqual(calls.path[0][0],p.path);
 }
});
test('local D* Lite flies, discovers obstacle, pauses to repair, then reaches goal',()=>{
 const draw=t=>frame(avoidance,'D* Lite replanning',t).calls;
 assert.deepEqual(p.repair_path[0],p.path[1]);
 assert.deepEqual(draw(0).drone[0][0],data.start);
 const moving=draw(.1);assert.equal(moving.box,undefined);
 assert.notDeepEqual(moving.drone[0][0],data.start);
 assert.notDeepEqual(moving.drone[0][0],p.repair_path[0]);
 for(const t of [.2,.4,.5,.6,.8]){
  const c=draw(t);assert.deepEqual(c.box[0].slice(0,2),p.obstacle);
  assert.deepEqual(c.drone[0][0],p.repair_path[0]);
  if(t<.6)assert.equal(c.path.length,1);
  else assert.deepEqual(c.path[1][0],p.repair_path);
 }
 assert.deepEqual(draw(.5).points[1][0],p.repair_visited.slice(0,Math.floor(.5*p.repair_visited.length)));
 const end=draw(1).drone[0][0];
 assert.ok(Math.hypot(...end.map((v,i)=>v-data.goal[i]))<1e-10);
});
