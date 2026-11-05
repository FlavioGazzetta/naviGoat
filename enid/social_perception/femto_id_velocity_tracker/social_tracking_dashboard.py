#!/usr/bin/env python3
"""Browser-based ROS 2 person-tracking viewer, separate from the YOLO pipeline.

Subscribes to /social_perception/tracks (std_msgs/String JSON).
Serves a zero-dependency HTML5 canvas dashboard on localhost:8765.

Run INSIDE the same ROS 2 Docker container as the tracker, with --network=host:
    python3 social_tracking_dashboard.py
Then from a Windows laptop:
    ssh -L 8765:127.0.0.1:8765 navigoat0@192.168.1.170
Browse http://localhost:8765

This viewer must not be used as a safety-critical obstacle detection interface.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Social perception | Live tracker</title>
<style>
  :root { color-scheme: dark; --bg: #0a111e; --panel: #142135; --panel2: #182840; --line: #2a3b54; --text: #f0f5fd; --muted: #a6bad2; --green: #65e0ae; --amber: #f4c76f; --blue: #8caaf8; }
  * { box-sizing: border-box; }
  body { background: var(--bg); color: var(--text); margin: 0; font-family: system-ui, -apple-system, 'Segoe UI', sans-serif; }
  .wrap { max-width: 1350px; padding: 24px; margin: auto; }
  .heading { display:flex; flex-wrap:wrap; justify-content:space-between; align-items:center; gap:12px; margin-bottom:20px; }
  h1 { margin: 0 0 4px; font-size:25px; font-weight:700; letter-spacing:-.6px; }
  .subtitle { margin:0; color:var(--muted); font-size:14px; }
  .live { display:flex; align-items:center; gap:9px; padding:8px 13px; border:1px solid var(--line); border-radius:22px; background:var(--panel); font-size:13px; color:var(--muted); }
  .led { width:9px; height:9px; border-radius:50%; background:#5b697d; }
  .led.on { background:var(--green); box-shadow: 0 0 10px #65e0ae75; }
  .cards { display:grid; grid-template-columns: repeat(3,minmax(0,1fr)); gap:12px; margin-bottom:14px; }
  .metric { background:var(--panel); border:1px solid var(--line); padding:14px 17px; border-radius:13px; }
  .metric span { display:block; color:var(--muted); font-size:12px; }
  .metric strong { display:block; margin-top:4px; font-size:23px; font-variant-numeric: tabular-nums; }
  .layout { display:grid; grid-template-columns: minmax(0,1.55fr) minmax(290px, .85fr); gap:14px; }
  .panel { background:var(--panel); border:1px solid var(--line); border-radius:14px; overflow:hidden; }
  .panel-title { font-size:14px; font-weight:650; margin:0; }
  .panel-head { padding:15px 17px; border-bottom:1px solid var(--line); display:flex; align-items:center; justify-content:space-between; gap:9px; flex-wrap:wrap; }
  .small { color:var(--muted); font-size:12px; }
  .mapbox { position:relative; padding:12px; }
  canvas { display:block; width:100%; height:auto; aspect-ratio: 1.25; background:#0d192a; border:1px solid var(--line); border-radius:10px; }
  .legend { display:flex; gap:16px; flex-wrap:wrap; padding:0 15px 14px; color:var(--muted); font-size:12px; }
  .sample { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:5px; vertical-align:middle; background:var(--green); }
  .sample.pred { background:transparent; border:2px dashed var(--amber); }
  .sample.future { border:2px solid var(--blue); background:transparent; }
  .controls { display:grid; grid-template-columns: repeat(3,1fr); gap:10px; padding:15px; border-bottom:1px solid var(--line); }
  label { display:block; font-size:12px; color:var(--muted); margin-bottom:6px; }
  select { width:100%; background:#0e1c30; color:var(--text); border:1px solid var(--line); border-radius:8px; padding:9px; }
  .people { padding:12px 15px; }
  .person { padding:12px; border-radius:9px; background:var(--panel2); margin-bottom:9px; border:1px solid var(--line); }
  .person-top { display:flex; justify-content:space-between; align-items:center; margin-bottom:8px; }
  .pill { color:var(--green); font-size:11px; border-radius:12px; padding:3px 8px; background:#24483f; }
  .pill.missing { color:var(--amber); background:#4a3b28; }
  .stats { display:grid; grid-template-columns: repeat(2,1fr); gap:8px; }
  .stat label { margin:0 0 3px; }
  .stat b { font-size:14px; font-variant-numeric: tabular-nums; font-weight:550; }
  .none { color:var(--muted); padding:24px 8px; text-align:center; }
  .foot { color:var(--muted); font-size:12px; line-height:1.5; padding:14px 16px; border-top:1px solid var(--line); }
  @media (max-width:900px) {.layout { grid-template-columns:1fr; } }
  @media (max-width:560px) {.wrap { padding:12px; } .cards {gap:6px;} .metric {padding:12px;} .metric strong {font-size:19px;} .controls {gap:6px;} }
</style>
</head>
<body>
<div class="wrap">
  <div class="heading"><div>
    <h1>Social perception</h1>
    <p class="subtitle">Orbbec Femto Bolt · Jetson Orin Nano · Live 2D person tracking</p>
  </div><div class="live"><i id="led" class="led"></i><span id="connection">Waiting for ROS data</span></div></div>
  <div class="cards">
    <div class="metric"><span>Confirmed tracks</span><strong id="trackcount">0</strong></div>
    <div class="metric"><span>Observed now</span><strong id="obsCount">0</strong></div>
    <div class="metric"><span>Viewer update rate</span><strong id="rate">–</strong></div>
  </div>
  <div class="layout">
    <section class="panel">
      <div class="panel-head"><h2 class="panel-title">Camera-relative map</h2><span class="small">Forward ↑ · Left ← · Right →</span></div>
      <div class="mapbox"><canvas id="map" width="900" height="720" aria-label="Top-down map of people"></canvas></div>
      <div class="legend">
        <span><i class="sample"></i> Observed</span>
        <span><i class="sample pred"></i> Kalman-predicted (temporarily lost)</span>
        <span><i class="sample future"></i> Future constant-velocity projection</span>
      </div>
    </section>
    <aside class="panel">
      <div class="panel-head"><h2 class="panel-title">Display settings</h2></div>
      <div class="controls">
        <div><label for="fwd">Forward range</label><select id="fwd"><option value="2">2 m</option><option value="3">3 m</option><option value="4" selected>4 m</option><option value="6">6 m</option><option value="8">8 m</option></select></div>
        <div><label for="lat">Side range (±)</label><select id="lat"><option value="1">1 m</option><option value="2" selected>2 m</option><option value="3">3 m</option><option value="5">5 m</option></select></div>
        <div><label for="horizon">Project ahead</label><select id="horizon"><option value="0">Off</option><option value="0.5" selected>0.5 s</option><option value="1">1.0 s</option><option value="1.5">1.5 s</option></select></div>
      </div>
      <div class="panel-head"><h2 class="panel-title">Tracks</h2><span class="small" id="sourceAge">No messages</span></div>
      <div class="people" id="people"><div class="none">Waiting for person tracks</div></div>
      <div class="foot">Filled dots are current detections. Dashed circles show Kalman predictions after a missed detection. The blue outline is a simple velocity-based estimate of position at the selected future time; it is not a learned trajectory prediction. The trail contains observed positions only. Coordinates assume a stationary, level camera.</div>
    </aside>
  </div>
</div>
<script>
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const $ = id => document.getElementById(id);
let state = { tracks: [], age_s: null };
let history = new Map();
let lastSeq = null, newSamples = [], lastPacketAt = 0;
const W = canvas.width, H = canvas.height;
const pad = {l:72,r:30,t:30,b:60};
let xMax=4, yMax=2, horizon=.5;
function project(x,y) {
  const plotW = W-pad.l-pad.r, plotH=H-pad.t-pad.b;
  return { x:pad.l + (yMax-y)/(2*yMax)*plotW, y:pad.t + (xMax-x)/xMax*plotH };
}
function line(x1,y1,x2,y2,color,width=1,dash=[]) {
  ctx.strokeStyle=color; ctx.lineWidth=width; ctx.setLineDash(dash);
  ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke(); ctx.setLineDash([]);
}
function text(s,x,y,color='#a6bad2',size=16,align='left') {
  ctx.fillStyle=color; ctx.font=`${size}px system-ui, sans-serif`; ctx.textAlign=align;
  ctx.fillText(s,x,y);
}
function circle(x,y,r,color,fill=true,dashed=false) {
  ctx.beginPath(); ctx.arc(x,y,r,0,Math.PI*2);
  if(fill){ctx.fillStyle=color;ctx.fill();} else {ctx.strokeStyle=color;ctx.lineWidth=3;ctx.setLineDash(dashed?[6,4]:[]);ctx.stroke();ctx.setLineDash([]);}
}
function arrow(a,b,color='#65e0ae',width=3) {
  let dx=b.x-a.x,dy=b.y-a.y,dist=Math.hypot(dx,dy);
  if(dist<4)return;
  line(a.x,a.y,b.x,b.y,color,width);
  let ang=Math.atan2(dy,dx),s=11;
  ctx.fillStyle=color;ctx.beginPath();ctx.moveTo(b.x,b.y);
  ctx.lineTo(b.x-s*Math.cos(ang-.48),b.y-s*Math.sin(ang-.48));
  ctx.lineTo(b.x-s*Math.cos(ang+.48),b.y-s*Math.sin(ang+.48));
  ctx.closePath();ctx.fill();
}
function draw() {
  xMax=Number($('fwd').value);yMax=Number($('lat').value);horizon=Number($('horizon').value);
  ctx.clearRect(0,0,W,H);ctx.fillStyle='#0d192a';ctx.fillRect(0,0,W,H);
  const p0=project(0,0);
  for(let i=0;i<=xMax*2;i++){
    let x=i*.5, p=project(x,0);line(pad.l,p.y,W-pad.r,p.y, '#233650');
    if(i%2===0)text(x+' m',pad.l-12,p.y+5,'#8197b4',15,'right');
  }
  let step=yMax<=2?.5:1;
  for(let y=-yMax;y<=yMax+.001;y+=step){
    let p=project(0,y);line(p.x,pad.t,p.x,H-pad.b,'#233650');
    text(y>0?'L '+y.toFixed(1):y<0?'R '+Math.abs(y).toFixed(1):'0',p.x,H-pad.b+25,'#8197b4',14,'center');
  }
  text('Distance forward',W/2,20,'#a6bad2',17,'center');
  text('Camera',p0.x,p0.y-21,'#8caaf8',15,'center');
  ctx.fillStyle='#8caaf8';ctx.fillRect(p0.x-10,p0.y-10,20,20);
  arrow({x:p0.x,y:p0.y-12},{x:p0.x,y:p0.y-44},'#8caaf8',3);
  const now=Date.now()/1000;
  for(const tr of state.tracks||[]) {
    const id=String(tr.id), points=history.get(id)||[];
    const recent=points.filter(p=>now-p.when<5);
    if(recent.length>1){
      ctx.beginPath();ctx.lineWidth=3;ctx.strokeStyle='#65e0ae65';
      recent.forEach((p,j)=>{let q=project(p.x,p.y); j===0?ctx.moveTo(q.x,q.y):ctx.lineTo(q.x,q.y);});
      ctx.stroke();
    }
    const p=project(tr.x,tr.y), observed=tr.observed===true;
    const color=observed?'#65e0ae':'#f4c76f';
    circle(p.x,p.y,12,color,observed,!observed);
    text('#'+tr.id,p.x+17,p.y-14,'#f0f5fd',19);
    if(horizon>0){
      const f=project(tr.x+tr.vx*horizon,tr.y+tr.vy*horizon);
      line(p.x,p.y,f.x,f.y,'#8caaf8',2,[6,5]);circle(f.x,f.y,7,'#8caaf8',false);
    }
    const velocityHorizon=.45;
    const dest=project(tr.x+tr.vx*velocityHorizon,tr.y+tr.vy*velocityHorizon);
    arrow(p,dest,color,3);
  }
}
function fmt(n,d=2){return typeof n==='number' && Number.isFinite(n) ? n.toFixed(d) : '–';}
function drawSidebar(){
  const trs=state.tracks||[];
  $('trackcount').textContent=trs.length;
  $('obsCount').textContent=trs.filter(t=>t.observed).length;
  $('sourceAge').textContent=state.age_s==null?'No messages':`Latest message ${fmt(state.age_s,1)} s ago`;
  const el=$('people'); el.replaceChildren();
  if(!trs.length){const n=document.createElement('div'); n.className='none';n.textContent='No confirmed people in view';el.append(n);return;}
  for(const t of trs){
    const p=document.createElement('div');p.className='person';
    const safe=(v,d=2)=>fmt(v,d);
    p.innerHTML=`<div class="person-top"><strong>Person #${Number(t.id)}</strong><span class="pill ${t.observed?'':'missing'}">${t.observed?'Detected':'Predicted'}</span></div>`+
      `<div class="stats"><div class="stat"><label>Forward</label><b>${safe(t.x)} m</b></div>`+
      `<div class="stat"><label>Left (+) / right (−)</label><b>${safe(t.y)} m</b></div>`+
      `<div class="stat"><label>Speed</label><b>${safe(Math.hypot(t.vx||0,t.vy||0))} m/s</b></div>`+
      `<div class="stat"><label>Velocity (forward, left)</label><b>${safe(t.vx)}, ${safe(t.vy)} m/s</b></div>`+
      `<div class="stat"><label>Detection matches</label><b>${Number(t.hits)||0}</b></div>`+
      `<div class="stat"><label>Time since observed</label><b>${safe(t.age_since_seen_s,1)} s</b></div></div>`;
    el.appendChild(p);
  }
}
let fetching=false;
async function poll(){
  if(fetching) return; fetching=true;
  try {
    const res=await fetch('/api/state',{cache:'no-store'});
    if(!res.ok)throw new Error('HTTP '+res.status);
    const s=await res.json();
    state=s;
    const active=s.age_s!==null && s.age_s<1.5;
    $('led').className='led'+(active?' on':'');
    $('connection').textContent=active?'ROS stream active':'Waiting for recent ROS data';
    if(s.seq!==lastSeq && active){
      lastSeq=s.seq; lastPacketAt=Date.now()/1000;
      newSamples.push(lastPacketAt);
      newSamples=newSamples.filter(x=>lastPacketAt-x<3);
      const hz=newSamples.length>2?(newSamples.length-1)/(newSamples[newSamples.length-1]-newSamples[0]):0;
      $('rate').textContent=hz?hz.toFixed(1)+' Hz':'–';
      const seen=new Set();
      for(const t of s.tracks||[]){
        const id=String(t.id);seen.add(id);
        if(t.observed){
          const a=history.get(id)||[];
          a.push({x:t.x,y:t.y,when:lastPacketAt});
          history.set(id,a.filter(p=>lastPacketAt-p.when<5).slice(-150));
        }
      }
      for(const id of history.keys()){
        if(!seen.has(id) && (!history.get(id).length || lastPacketAt-history.get(id).at(-1).when>5)) history.delete(id);
      }
    }
    if(!active){state={...state,tracks:[]};$('rate').textContent='–';}
    drawSidebar();
  } catch(e){$('led').className='led';$('connection').textContent='Viewer disconnected';state={tracks:[],age_s:null};drawSidebar();}
  finally {fetching=false;draw();}
}
for(const id of ['fwd','lat','horizon'])$(id).addEventListener('change',draw);
setInterval(poll,125);poll();draw();
</script></body></html>'''


class TrackViewer(Node):
    def __init__(self, topic):
        super().__init__("social_tracking_dashboard")
        self._lock = threading.Lock()
        self._message = {"tracks": []}
        self._received_at = None
        self._seq = 0
        self.create_subscription(String, topic, self.on_tracks, 10)
        self.get_logger().info(f"Subscribed to {topic}")

    def on_tracks(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("JSON payload is not an object")
            tracks = payload.get("tracks", [])
            if not isinstance(tracks, list):
                raise ValueError("tracks must be a list")
            # Avoid forwarding unneeded payload fields.
            slim = []
            for t in tracks:
                if not isinstance(t, dict):
                    continue
                slim.append({k: t.get(k) for k in
                             ("id", "x", "y", "vx", "vy", "hits", "observed", "confirmed", "age_since_seen_s")})
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignored invalid tracking JSON: {exc}")
            return
        with self._lock:
            self._message = {"tracks": slim, "stamp_sec": payload.get("stamp_sec")}
            self._received_at = time.monotonic()
            self._seq += 1

    def snapshot(self):
        with self._lock:
            age = None if self._received_at is None else time.monotonic() - self._received_at
            return {"tracks": list(self._message["tracks"]),
                    "stamp_sec": self._message.get("stamp_sec"),
                    "age_s": age, "seq": self._seq}


class Handler(BaseHTTPRequestHandler):
    viewer = None

    def do_GET(self):
        path = self.path.split('?',1)[0]
        if path == '/':
            content = HTML.encode('utf-8')
            typ = 'text/html; charset=utf-8'
        elif path == '/api/state':
            content = json.dumps(self.viewer.snapshot(), allow_nan=False).encode('utf-8')
            typ = 'application/json; charset=utf-8'
        else:
            self.send_error(404, 'Not found')
            return
        self.send_response(200)
        self.send_header('Content-Type', typ)
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, *_args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--host', default='127.0.0.1',
                        help='Bind address; keep 127.0.0.1 when using an SSH tunnel')
    parser.add_argument('--topic', default='/social_perception/tracks')
    args = parser.parse_args()
    rclpy.init()
    viewer = TrackViewer(args.topic)
    Handler.viewer = viewer
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Viewer: http://{args.host}:{args.port} (SSH tunnel recommended)", flush=True)
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
