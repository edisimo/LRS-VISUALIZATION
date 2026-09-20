import test from 'node:test';
import assert from 'node:assert/strict';
import {distance,voxelize,octree,interpolate} from '../common/math.js';
test('signed box distance preserves inside and outside meaning',()=>{let b=[[[0,0,0],[2,2,2]]];assert.equal(distance([1,1,1],b),-1);assert.equal(distance([3,1,1],b),1);assert.equal(distance([3,3,2],b),Math.SQRT2);});
test('voxel storage deduplicates cells and respects crop',()=>{assert.equal(voxelize([[.1,.1,.1],[.2,.2,.2],[2,2,2]],1,[[0,0,0],[1,1,1]]).length,1);});
test('octree retains empty siblings and covers the entire root volume',()=>{let t=octree([[1,3,1]],3);assert.equal(t.leaves.reduce((s,l)=>s+l.size**3,0),1000);assert.equal(t.nodes,25);assert.equal(t.leaves.filter(l=>l.occupied).length,1);});
test('interpolation includes both endpoints',()=>{assert.deepEqual(interpolate([[0,0,0],[1,2,3]],0),[0,0,0]);assert.deepEqual(interpolate([[0,0,0],[1,2,3]],1),[1,2,3]);});
