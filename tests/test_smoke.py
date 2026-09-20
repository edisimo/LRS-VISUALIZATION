"""No third-party dependencies: validates distributed caches and launchers."""
import importlib.util
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time
import unittest
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
D=json.loads((ROOT/'precomputed/lecture.json').read_text())

def collision_free(a,b,boxes,r=.35):
    for lo,hi in boxes:
        enter,leave=0.,1.
        for i in range(3):
            velocity=b[i]-a[i]
            if abs(velocity)<1e-10:
                if a[i]<lo[i]-r or a[i]>hi[i]+r:break
            else:
                t0=(lo[i]-r-a[i])/velocity;t1=(hi[i]+r-a[i])/velocity
                enter=max(enter,min(t0,t1));leave=min(leave,max(t0,t1))
                if enter>leave:break
        else:return False
    return all(D['bounds'][0][k]+r-1e-4<=p[k]<=D['bounds'][1][k]-r+1e-4 for p in [a,b] for k in range(3))

class Smoke(unittest.TestCase):
    def test_assets(self):
        meta=json.loads((ROOT/'assets/provenance.json').read_text())
        self.assertEqual((ROOT/'assets/hangar-points.bin').stat().st_size,meta['display_points']*12)
        self.assertEqual(meta['original_points'],620311)
        for file in ['app.js','mapping/view.js','planning/view.js','trajectory/view.js','avoidance/view.js','full_pipeline/view.js','vendor/three.module.min.js','vendor/three.core.min.js','vendor/OrbitControls.js']:
            self.assertTrue((ROOT/file).is_file(),file)
    def test_planners(self):
        for name,p in D['planners'].items():
            with self.subTest(planner=name):
                self.assertEqual(p['path'][0],D['start']);self.assertEqual(p['path'][-1],D['goal'])
                self.assertTrue(all(collision_free(a,b,D['boxes']) for a,b in zip(p['path'][:-1],p['path'][1:])))
                self.assertGreater(p['ms'],0)
                if 'best' in p:
                    costs=[v['length'] for v in p['best']];self.assertTrue(all(a>=b for a,b in zip(costs,costs[1:])))
        self.assertAlmostEqual(D['planners']['Dijkstra']['length'],D['planners']['A*']['length'],places=4)
        self.assertLess(D['planners']['A*']['nodes'],D['planners']['Dijkstra']['nodes'])
        self.assertTrue(any(e['rewires'] for e in D['planners']['RRT*']['events']))
        informed=D['planners']['Informed RRT*']
        for n,event in enumerate(informed['events'],1):
            previous=[b for b in informed['best'] if b['at']<n]
            if previous:
                sample=event['sample']
                self.assertLessEqual(math.dist(sample,D['start'])+math.dist(sample,D['goal']),previous[-1]['length']+1e-3)
    def test_trajectory(self):
        tr=D['trajectory'];self.assertLess(tr['spline_clearance'],D['radius']);self.assertLess(len(tr['pruned']),len(tr['raw']))
        self.assertLess(tr['snap']['constraint_error'],1e-6)
        self.assertLess(tr['jerk']['constraint_error'],1e-6)
        self.assertLess(tr['chomp'][-1]['cost'],tr['chomp'][0]['cost'])
        for name in ['snap','jerk']:
            for derivative in ['velocity','acceleration']:
                self.assertLess(math.dist(tr[name][derivative][0],[0,0,0]),1e-4)
                self.assertLess(math.dist(tr[name][derivative][-1],[0,0,0]),1e-4)
    def test_avoidance(self):
        av=D['avoidance'];boxes=D['boxes']+[av['obstacle']]
        self.assertTrue(any(not collision_free(a,b,boxes) for a,b in zip(D['trajectory']['pruned'][:-1],D['trajectory']['pruned'][1:])))
        for h,controllers in av['controllers'].items():
            for name,c in controllers.items():
                with self.subTest(horizon=h,controller=name):
                    self.assertTrue(c['reached'])
                    self.assertTrue(all(collision_free(a,b,boxes) for a,b in zip(c['actual'][:-1],c['actual'][1:])))
                for i,frame in enumerate(c['frames']):
                    expected=[v+c['dt']*u for v,u in zip(frame['position'],frame['control'])]
                    self.assertLess(math.dist(expected,c['actual'][i+1]),1e-4)
        self.assertEqual(av['execution'][0],av['detection'])
        self.assertEqual(av['execution'][-1],D['goal'])
        self.assertTrue(all(collision_free(a,b,boxes) for a,b in zip(av['execution'][:-1],av['execution'][1:])))
    def test_launchers(self):
        for name in ['mapping','planning','trajectory','avoidance','full_demo']:
            with self.subTest(launcher=name):
                with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
                proc=subprocess.Popen([str(ROOT/f'run_{name}.sh'),'--no-browser','--port',str(port)],cwd='/tmp',stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                try:
                    for _ in range(50):
                        try:
                            with urllib.request.urlopen(f'http://127.0.0.1:{port}/',timeout=.2) as response:self.assertIn(b'LRS / Flight Lab',response.read())
                            break
                        except OSError:time.sleep(.05)
                    else:self.fail(f'Launcher failed: {proc.poll()}')
                finally:proc.terminate();proc.communicate(timeout=3)
    def test_launcher_import(self):
        spec=importlib.util.spec_from_file_location('serve',ROOT/'scripts/serve.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

if __name__=='__main__':unittest.main()
