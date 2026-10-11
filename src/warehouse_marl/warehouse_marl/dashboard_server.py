"""
dashboard_server.py
--------------------
Tableau de bord web du systeme : carte vivante de l'entrepot extraite du
monde Gazebo, robots, lectures RFID (tags lus, racks scannes, missions de
scan), missions et consensus.

    ros2 run warehouse_marl warehouse_dashboard      # puis http://localhost:8080

Aucune dependance externe (pas de rosbridge, Node.js ni base de donnees) :
ce noeud ROS 2 sert la page (HTTP) et pousse l'etat en continu au
navigateur (Server-Sent Events). Les actions de l'operateur passent par les
memes topics que le reste du systeme :
  - "Demander un scan RFID" -> /rfid/scan_requests -> tache rfid_scan
  - "Creer une tache de picking" -> /warehouse/tasks/new -> consensus
"""

import json
import math
import os
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import tf2_ros

from warehouse_marl.rfid_model import rack_faces
from warehouse_marl.warehouse_graph import build_demo_warehouse

TERMINAL = ('succeeded', 'failed', 'rejected', 'canceled')
CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
                 '.js': 'application/javascript; charset=utf-8', '.svg': 'image/svg+xml'}


# --------------------------------------------------------------------------- #
# Monde Gazebo -> entites 2D (meme principe que extract-sdf-worlds.mjs d'AMR-X)
# --------------------------------------------------------------------------- #

def _category(name: str) -> str:
    n = name.lower()
    if n.startswith('wall') or n.startswith('sep_'):
        return 'wall'
    if n.startswith('sh_'):
        return 'rack'
    if n.startswith('zone'):
        return 'zone'
    if 'charger' in n or n.startswith('dock'):
        return 'dock'
    if 'pallet' in n or n.startswith('box'):
        return 'payload'
    return 'obstacle'


def _pose(text):
    v = [float(x) for x in (text or '').split()] + [0.0] * 6
    return v[0], v[1], v[5]


def parse_world(path: str):
    """Entites 2D (repere monde) d'un fichier .world / .sdf."""
    root = ET.parse(path).getroot()
    world = root.find('world')
    entities = []
    for model in world.findall('model'):
        name = model.get('name', '')
        if name == 'ground_plane' or name.endswith('_upper') or 'monitor_cam' in name:
            continue
        x, y, yaw = _pose(model.findtext('pose'))
        geom = None
        for link in model.findall('link'):
            for col in link.findall('collision'):
                box = col.findtext('geometry/box/size')
                cyl = col.findtext('geometry/cylinder/radius')
                if box:
                    sx, sy, _ = (float(v) for v in box.split())
                    geom = {'shape': 'box', 'sx': sx, 'sy': sy}
                elif cyl:
                    geom = {'shape': 'cyl', 'r': float(cyl)}
                if geom:
                    break
            if geom:
                break
        if geom is None:
            continue
        entities.append({'name': name, 'category': _category(name),
                         'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4), **geom})
    for inc in world.findall('include'):
        x, y, yaw = _pose(inc.findtext('pose'))
        name = inc.findtext('name') or (inc.findtext('uri') or 'include').rsplit('/', 1)[-1]
        entities.append({'name': name, 'category': 'actor', 'x': x, 'y': y, 'yaw': yaw,
                         'shape': 'cyl', 'r': 0.3})
    return world.get('name', 'world'), entities


# --------------------------------------------------------------------------- #
# Noeud ROS
# --------------------------------------------------------------------------- #

class DashboardNode(Node):

    def __init__(self):
        super().__init__('warehouse_dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('map_origin_x', -1.5)
        self.declare_parameter('map_origin_y', -1.0)
        self.declare_parameter('world_file', '')

        self.robot_names = list(self.get_parameter('robot_names').value)
        self.ox = float(self.get_parameter('map_origin_x').value)
        self.oy = float(self.get_parameter('map_origin_y').value)
        self.lock = threading.Lock()

        world_file = str(self.get_parameter('world_file').value)
        if not world_file:
            world_file = os.path.join(
                get_package_share_directory('rosnav_bot'), 'worlds', 'warehouse.world')
        world_name, entities = parse_world(world_file)
        faces = rack_faces(0.0, 0.0)  # repere monde
        graph = build_demo_warehouse(0.0, 0.0)
        self.static = {
            'world': world_name,
            'entities': entities,
            'faces': {s: {'x0': f.x0, 'x1': f.x1, 'y': f.y} for s, f in faces.items()},
            'nodes': {n: {'x': d['x'], 'y': d['y'], 'type': d['node_type']}
                      for n, d in graph.G.nodes(data=True)},
            'robots': self.robot_names,
            'map_origin': [self.ox, self.oy],
        }

        self.robots = {r: {'x': None, 'y': None, 'yaw': 0.0, 'nav_busy': False,
                           'last_seen': None, 'reads': [], 'read_stamp': None}
                       for r in self.robot_names}
        self.tag_shelf = {}          # tag -> rack where the tag physically is
        self.tag_last = {}           # tag -> (wall time, robot) of its last read
        self.shelf_reads = {s: 0 for s in self.static['faces']}
        self.shelf_last = {}         # rack -> (wall time, robot) of its last read
        self.total_reads = 0
        self.reads_history = deque(maxlen=900)   # [sim time, reads/s]
        self._bucket = [None, 0]                 # [bucket start sim time, reads]
        self.recent_reads = deque(maxlen=40)
        self.sim_start = None
        self.sim_now = None
        self.tasks = {}
        self.task_order = deque(maxlen=200)
        self.activity = deque(maxlen=60)
        self.scan_requests = {}
        self.exploration = {}

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self.create_timer(0.2, self._update_poses)

        self.create_subscription(String, '/rfid/ground_truth', self._on_truth, 10)
        self.create_subscription(String, '/rfid/scans', self._on_scan, 50)
        self.create_subscription(String, '/rfid/scan_requests', self._on_scan_request, 10)
        self.create_subscription(String, '/warehouse/tasks/new', self._on_task, 20)
        self.create_subscription(String, '/warehouse/consensus/assignments', self._on_claim, 20)
        self.create_subscription(String, '/warehouse/tasks/status', self._on_status, 50)
        self.create_subscription(String, '/exploration/stats', self._on_exploration, 10)
        status_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        busy = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING)
        for r in self.robot_names:
            self.create_subscription(
                GoalStatusArray, f'/{r}/navigate_to_pose/_action/status',
                lambda m, r=r: self._set(r, 'nav_busy', any(s.status in busy for s in m.status_list)),
                status_qos)

        self.scan_pub = self.create_publisher(String, '/rfid/scan_requests', 10)
        self.task_pub = self.create_publisher(String, '/warehouse/tasks/new', 10)

    # -- helpers -----------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set(self, robot, key, value):
        with self.lock:
            self.robots[robot][key] = value

    def _event(self, kind, text):
        # Wall-clock time: the browser shows "x s ago" against wall time.
        self.activity.appendleft({'t': time.time(), 'kind': kind, 'text': text})

    # -- ROS callbacks -----------------------------------------------------
    def _update_poses(self):
        for r in self.robot_names:
            try:
                tf = self._tf.lookup_transform('map', f'{r}/base_link', rclpy.time.Time())
            except Exception:
                continue
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self.lock:
                rob = self.robots[r]
                rob['x'] = round(tf.transform.translation.x + self.ox, 3)
                rob['y'] = round(tf.transform.translation.y + self.oy, 3)
                rob['yaw'] = round(yaw, 3)
                rob['last_seen'] = time.time()

    def _on_truth(self, msg):
        # Only used to know on which rack each tag physically sits.
        data = json.loads(msg.data)
        with self.lock:
            self.tag_shelf = data.get('truth', {})

    def _on_scan(self, msg):
        data = json.loads(msg.data)
        r = data.get('robot_id')
        if r not in self.robots:
            return
        now = time.time()
        stamp = float(data.get('stamp', 0.0))
        tags = [x['tag'] for x in data.get('reads', [])]
        with self.lock:
            self.robots[r]['reads'] = tags
            self.robots[r]['read_stamp'] = now
            self.sim_now = stamp
            if self.sim_start is None:
                self.sim_start = stamp
            shelves = set()
            for tag in tags:
                self.total_reads += 1
                self.tag_last[tag] = (now, r)
                shelf = self.tag_shelf.get(tag)
                if shelf in self.shelf_reads:
                    self.shelf_reads[shelf] += 1
                    self.shelf_last[shelf] = (now, r)
                    shelves.add(shelf)
            if tags:
                self.recent_reads.appendleft({
                    't': now, 'robot': r, 'count': len(tags),
                    'shelves': sorted(shelves), 'tags': tags[:6]})
            # Reads per second, in 5 s buckets of simulation time.
            start, count = self._bucket
            if start is None:
                self._bucket = [stamp, len(tags)]
            elif stamp - start >= 5.0:
                self.reads_history.append([round(start, 1), round(count / (stamp - start), 2)])
                self._bucket = [stamp, len(tags)]
            else:
                self._bucket[1] += len(tags)

    def _on_scan_request(self, msg):
        data = json.loads(msg.data)
        shelf = data.get('shelf')
        with self.lock:
            self.scan_requests[shelf] = time.time()
            src = 'operateur' if data.get('source') == 'dashboard' else 'automatique'
            self._event('scan', f'Scan RFID demande sur {shelf} ({src})')

    def _on_task(self, msg):
        data = json.loads(msg.data)
        tid = data.get('task_id')
        with self.lock:
            self.tasks[tid] = {'id': tid, 'type': data.get('task_type', 'pick'),
                               'target': data.get('target_node'),
                               'priority': data.get('priority'), 'state': 'auction',
                               'robot': None, 'claims': {}, 'created': time.time(),
                               'elapsed': None, 'retries': 0}
            self.task_order.appendleft(tid)
            if len(self.tasks) > 400:
                keep = set(self.task_order)
                self.tasks = {k: v for k, v in self.tasks.items() if k in keep}
            self._event('task', f'Nouvelle tache {data.get("task_type", "pick")} -> {data.get("target_node")}')

    def _on_claim(self, msg):
        data = json.loads(msg.data)
        tid = data.get('task_id')
        with self.lock:
            task = self.tasks.get(tid)
            if task is None:
                return
            task['claims'][data.get('robot_id')] = data.get('cost')
            claims = task['claims']
            task['robot'] = min(claims, key=lambda r: (claims[r] if claims[r] is not None else 1e9, r))
            if task['state'] == 'auction':
                task['state'] = 'assigned'
            if len(claims) > 1:
                self._event('consensus', f'Conflit {tid} : {len(claims)} revendications, retenu {task["robot"]}')
            else:
                self._event('consensus', f'{data.get("robot_id")} gagne {tid} (cout {data.get("cost", 0):.2f})')

    def _on_status(self, msg):
        data = json.loads(msg.data)
        tid = data.get('task_id')
        with self.lock:
            task = self.tasks.get(tid)
            if task is None:
                return
            task['state'] = data.get('state')
            task['robot'] = data.get('robot_id') or task['robot']
            task['retries'] = data.get('retries', 0)
            if data.get('elapsed_sec') is not None:
                task['elapsed'] = round(data['elapsed_sec'], 1)
            if task['state'] in TERMINAL:
                verb = 'termine' if task['state'] == 'succeeded' else 'echoue'
                self._event('done' if task['state'] == 'succeeded' else 'fail',
                            f'{task["robot"]} a {verb} {task["type"]} {task["target"]}')

    def _on_exploration(self, msg):
        with self.lock:
            self.exploration = json.loads(msg.data)

    # -- snapshot / actions ------------------------------------------------
    def snapshot(self):
        now = time.time()
        with self.lock:
            robots = {}
            for r, d in self.robots.items():
                fresh_reads = d['read_stamp'] and now - d['read_stamp'] < 2.5
                robots[r] = {'x': d['x'], 'y': d['y'], 'yaw': d['yaw'],
                             'online': bool(d['last_seen'] and now - d['last_seen'] < 3.0),
                             'nav_busy': d['nav_busy'],
                             'reads': d['reads'] if fresh_reads else []}
            for t in self.tasks.values():
                if t['state'] not in TERMINAL and t['robot'] in robots:
                    robots[t['robot']]['task'] = {'id': t['id'], 'type': t['type'],
                                                  'target': t['target'], 'state': t['state']}
            tasks = [self.tasks[t] for t in list(self.task_order)[:80] if t in self.tasks]
            counts = {'open': 0, 'succeeded': 0, 'failed': 0, 'scans_done': 0, 'scans_total': 0}
            for t in self.tasks.values():
                if t['state'] == 'succeeded':
                    counts['succeeded'] += 1
                elif t['state'] in TERMINAL:
                    counts['failed'] += 1
                else:
                    counts['open'] += 1
                if t['type'] == 'rfid_scan':
                    counts['scans_total'] += 1
                    counts['scans_done'] += t['state'] == 'succeeded'
            shelves = {}
            for shelf in self.static['faces']:
                tags_here = [t for t, sh in self.tag_shelf.items() if sh == shelf]
                last = self.shelf_last.get(shelf)
                shelves[shelf] = {
                    'tags': len(tags_here),
                    'read_recent': sum(1 for t in tags_here
                                       if t in self.tag_last and now - self.tag_last[t][0] < 60),
                    'reads': self.shelf_reads[shelf],
                    'last_read_age': None if last is None else round(now - last[0], 1),
                    'last_robot': None if last is None else last[1],
                }
            tags = {t: {'shelf': sh,
                        'last_read_age': None if t not in self.tag_last else round(now - self.tag_last[t][0], 1),
                        'robot': self.tag_last.get(t, (None, None))[1]}
                    for t, sh in self.tag_shelf.items() if sh is not None}
            rate = self.reads_history[-1][1] if self.reads_history else 0.0
            return {
                'time': now,
                'robots': robots,
                'rfid': {
                    'total_reads': self.total_reads,
                    'tags_total': len(tags),
                    'tags_seen': sum(1 for t in tags if t in self.tag_last),
                    'reads_per_s': rate,
                    'shelves': shelves,
                    'recent': list(self.recent_reads),
                    'history': list(self.reads_history)[-360:],
                },
                'tags': tags,
                'sim_elapsed': None if self.sim_start is None else round(self.sim_now - self.sim_start, 1),
                'tasks': tasks,
                'task_counts': counts,
                'activity': list(self.activity)[:40],
                'scan_requests': {s: now - t for s, t in self.scan_requests.items() if now - t < 60},
                'exploration': {k: self.exploration.get(k) for k in
                                ('explored_pct', 'succeeded_goals', 'failed_goals', 'tf_ready')},
            }

    def request_scan(self, shelf):
        if shelf not in self.static['faces']:
            return False
        msg = String()
        msg.data = json.dumps({'shelf': shelf, 'score': 9.0, 'stamp': self._now(),
                               'source': 'dashboard'})
        self.scan_pub.publish(msg)
        return True

    def create_pick(self, shelf, priority):
        if shelf not in self.static['faces']:
            return None
        task_id = f'task_ui_{uuid.uuid4().hex[:6]}'
        msg = String()
        msg.data = json.dumps({'task_id': task_id, 'task_type': 'pick', 'target_node': shelf,
                               'priority': float(priority), 'created_at': self._now()})
        self.task_pub.publish(msg)
        return task_id


# --------------------------------------------------------------------------- #
# Serveur HTTP + Server-Sent Events
# --------------------------------------------------------------------------- #

def make_handler(node: DashboardNode, web_root: str):

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, payload, code=200):
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/static':
                return self._json(node.static)
            if path == '/api/state':
                return self._json(node.snapshot())
            if path == '/api/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                try:
                    while rclpy.ok():
                        self.wfile.write(f'data: {json.dumps(node.snapshot())}\n\n'.encode())
                        self.wfile.flush()
                        time.sleep(0.5)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            name = 'index.html' if path in ('/', '') else path.lstrip('/')
            # abspath (not realpath): --symlink-install makes these files
            # symlinks into src/, which is fine; only block '..' traversal.
            root = os.path.abspath(web_root)
            full = os.path.abspath(os.path.join(root, name))
            if not full.startswith(root + os.sep) or not os.path.isfile(full):
                self.send_error(404)
                return
            with open(full, 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header('Content-Type', CONTENT_TYPES.get(os.path.splitext(full)[1],
                                                              'application/octet-stream'))
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            length = int(self.headers.get('Content-Length') or 0)
            try:
                data = json.loads(self.rfile.read(length) or b'{}')
            except json.JSONDecodeError:
                return self._json({'ok': False, 'error': 'JSON invalide'}, 400)
            if self.path == '/api/scan':
                ok = node.request_scan(data.get('shelf'))
                return self._json({'ok': ok}, 200 if ok else 400)
            if self.path == '/api/pick':
                tid = node.create_pick(data.get('shelf'), data.get('priority', 1.5))
                return self._json({'ok': bool(tid), 'task_id': tid}, 200 if tid else 400)
            self.send_error(404)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    web_root = os.path.join(get_package_share_directory('warehouse_marl'), 'dashboard')
    port = int(node.get_parameter('port').value)
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node, web_root))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'Tableau de bord : http://localhost:{port}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
