"""Upgrade lecture data without recalculating unchanged source assets/controllers."""
import sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from common.algorithms import graph_search
from common.extensions import prm,dstar_lite,stomp,local_methods
from scripts.precompute import dump

def refresh(data):
    for name in ['Dijkstra','A*','Theta*','Weighted A*']:
        data['planners'][name]=graph_search(name,2.5 if name=='Weighted A*' else 1,connectivity=26)
        print(name,data['planners'][name]['nodes'],flush=True)
    data['planners']['PRM']=prm();print('PRM ready',flush=True)
    data['planners']['D* Lite']=dstar_lite(data['avoidance']['obstacle']);print('D* Lite ready',flush=True)
    data['trajectory']['stomp']=stomp(data['trajectory']['raw']);print('STOMP ready',flush=True)
    data['avoidance']['local']=local_methods(data['avoidance']['obstacle'],data['trajectory']['pruned']);print('Local methods ready',flush=True)
    data['version']=3
    return data

if __name__=='__main__':dump(ROOT/'precomputed/lecture.json',refresh(json.loads((ROOT/'precomputed/lecture.json').read_text())))
