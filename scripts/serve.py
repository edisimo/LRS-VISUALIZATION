#!/usr/bin/env python3
"""Standard-library-only offline launcher, bound to loopback."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import threading
import webbrowser

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic',choices=['mapping','planning','trajectory','avoidance'],default='mapping')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--no-browser',action='store_true')
    args=parser.parse_args()
    for asset in ['precomputed/lecture.json','assets/hangar-points.bin','vendor/three.module.min.js']:
        if not (ROOT/asset).is_file():parser.error(f'Missing {asset}; restore the distributed assets.')
    class Handler(SimpleHTTPRequestHandler):
        def log_message(self,*args):pass
    handler=partial(Handler,directory=str(ROOT))
    try:server=ThreadingHTTPServer(('127.0.0.1',args.port),handler)
    except OSError as exc:parser.exit(1,f'Cannot listen on port {args.port}: {exc}\nChoose --port 8766 or use the already-open lab tab.\n')
    url=f'http://127.0.0.1:{server.server_port}/#{args.topic}'
    print(f'LRS Flight Lab → {url}\nOffline assets ready. Ctrl+C to stop.',flush=True)
    if not args.no_browser:threading.Timer(.25,lambda:webbrowser.open(url)).start()
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
