'use strict';
/* Tableau de bord Warehouse MARL : carte vivante extraite du monde Gazebo,
   lectures RFID des robots, missions et consensus. Donnees : GET /api/static
   + flux SSE /api/stream (dashboard_server.py). */

const ROBOT_COLORS = ['#7c8cff', '#4ade80', '#ff8a3d', '#f472b6', '#22d3ee', '#facc15'];
const ENTITY_COLORS = {            // materiaux de l'entrepot
  wall: '#6b7280', rack: '#9c7a4f', zone: '#3b82f6', dock: '#eab308',
  payload: '#b45309', obstacle: '#94a3b8', actor: '#f472b6',
};
const FOOTPRINT = [[0.46, 0.25], [0.46, -0.25], [-0.28, -0.25], [-0.28, 0.25]];
const TITLES = {
  overview: ['Operations', "Vue d'ensemble"],
  map: ['Operations', 'Carte en direct'],
  missions: ['Operations', 'Missions & consensus'],
  reads: ['RFID', 'Lectures RFID'],
};
const RECENT_SCAN_SEC = 30;     // rack "scanne recemment"
const FRESH_TAG_SEC = 3;        // tag "lu a l'instant"

let STATIC = null;
let STATE = null;
let view = 'overview';
let selected = null;
const layers = { tags: true, beams: true, trails: true, graph: false };
const trails = {};
const $ = (id) => document.getElementById(id);

// ------------------------------------------------------------------ utils
function setHTML(el, html) { if (el && el.__html !== html) { el.innerHTML = html; el.__html = html; } }
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
// Racks are labelled E<row>-<col> (etagere) so they never look like robots (R1..Rn).
const rackLabel = (s) => (s ? s.replace('shelf_', 'E').replace('_', '-') : '—');
const robotColor = (r) => ROBOT_COLORS[STATIC.robots.indexOf(r) % ROBOT_COLORS.length];
function fmtClock(sec) {
  if (sec == null) return '—';
  const m = Math.floor(sec / 60), s = Math.floor(sec % 60);
  return `T+ ${m}:${String(s).padStart(2, '0')}`;
}
function ago(t) { const d = Math.max(0, STATE.time - t); return fmtAge(d); }
function fmtAge(d) { return d == null ? 'jamais' : d < 60 ? `${d.toFixed(0)} s` : `${(d / 60).toFixed(0)} min`; }
function toast(text) {
  const t = $('toast'); t.textContent = text; t.classList.add('show');
  clearTimeout(t.__h); t.__h = setTimeout(() => t.classList.remove('show'), 2600);
}
async function post(url, body) {
  try {
    const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    return r.ok;
  } catch { return false; }
}

// ------------------------------------------------------------------ geometry
let B = null; // bornes du monde
const S = 26; // px par metre
function computeBounds() {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const e of STATIC.entities) {
    if (e.category !== 'wall') continue;
    const hx = (e.sx || 0) / 2, hy = (e.sy || 0) / 2;
    minX = Math.min(minX, e.x - hx); maxX = Math.max(maxX, e.x + hx);
    minY = Math.min(minY, e.y - hy); maxY = Math.max(maxY, e.y + hy);
  }
  const m = 0.6;
  B = { minX: minX - m, maxX: maxX + m, minY: minY - m, maxY: maxY + m };
  B.w = (B.maxX - B.minX) * S; B.h = (B.maxY - B.minY) * S;
}
const X = (x) => (x - B.minX) * S;
const Y = (y) => (B.maxY - y) * S;
const deg = (yaw) => (-yaw * 180) / Math.PI;

function tagPositions() {
  // Position d'affichage de chaque tag le long de la face de son rack.
  const byShelf = {};
  for (const [tag, d] of Object.entries(STATE.tags)) (byShelf[d.shelf] ||= []).push(tag);
  const pos = {};
  for (const [shelf, tags] of Object.entries(byShelf)) {
    const f = STATIC.faces[shelf]; if (!f) continue;
    tags.sort();
    tags.forEach((tag, i) => { pos[tag] = [f.x0 + ((i + 1) * (f.x1 - f.x0)) / (tags.length + 1), f.y]; });
  }
  return pos;
}

// ------------------------------------------------------------------ map
function faceState(shelf) {
  if (STATE.scan_requests[shelf] != null) return 'scan';
  const age = STATE.rfid.shelves[shelf]?.last_read_age;
  return age != null && age < RECENT_SCAN_SEC ? 'recent' : 'old';
}
const FACE_COLORS = { recent: '#4ade80', old: '#4b4d55', scan: '#818cf8' };

function renderMap(compact) {
  const p = [];
  p.push(`<svg viewBox="0 0 ${B.w.toFixed(0)} ${B.h.toFixed(0)}" xmlns="http://www.w3.org/2000/svg">`);
  p.push(`<defs><pattern id="grid" width="${S}" height="${S}" patternUnits="userSpaceOnUse">
    <path d="M ${S} 0 L 0 0 0 ${S}" fill="none" stroke="rgba(255,255,255,.05)" stroke-width="1"/></pattern></defs>`);
  p.push(`<rect width="100%" height="100%" fill="url(#grid)"/>`);

  // Entites du monde Gazebo (zones d'abord)
  const order = ['zone', 'dock', 'payload', 'obstacle', 'actor', 'rack', 'wall'];
  for (const cat of order) {
    for (const e of STATIC.entities) {
      if (e.category !== cat) continue;
      const c = ENTITY_COLORS[cat] || '#6b7280';
      if (e.shape === 'cyl') {
        p.push(`<circle cx="${X(e.x)}" cy="${Y(e.y)}" r="${Math.max(2, e.r * S)}" fill="${c}" opacity="${cat === 'actor' ? .55 : .8}"/>`);
        continue;
      }
      const w = e.sx * S, h = e.sy * S;
      const op = cat === 'zone' ? 0.05 : cat === 'rack' ? 0.35 : 0.75;
      p.push(`<rect x="${-w / 2}" y="${-h / 2}" width="${w}" height="${h}" fill="${c}" fill-opacity="${op}" stroke="${c}" stroke-opacity="${cat === 'zone' ? .25 : .6}" transform="translate(${X(e.x)},${Y(e.y)}) rotate(${deg(e.yaw)})" rx="${cat === 'rack' ? 2 : 1}"/>`);
      if (cat === 'zone' && !compact) {
        p.push(`<text class="map-label" x="${X(e.x - e.sx / 2) + 6}" y="${Y(e.y + e.sy / 2) + 12}">${esc(e.name.replace('zone_', '').toUpperCase())}</text>`);
      }
    }
  }

  // Graphe logique
  if (layers.graph && !compact) {
    for (const [n, d] of Object.entries(STATIC.nodes)) {
      p.push(`<circle cx="${X(d.x)}" cy="${Y(d.y)}" r="3" fill="#ffb27a" opacity=".6"/>`);
      p.push(`<text class="map-label" x="${X(d.x) + 5}" y="${Y(d.y) - 4}">${esc(n)}</text>`);
    }
  }

  // Faces de rack : scannees recemment / pas recemment / scan demande
  for (const [shelf, f] of Object.entries(STATIC.faces)) {
    const st = faceState(shelf);
    const sel = selected === shelf ? ' selected' : '';
    p.push(`<rect class="face${sel}" data-shelf="${shelf}" x="${X(f.x0)}" y="${Y(f.y) - 4}" width="${(f.x1 - f.x0) * S}" height="8" rx="3" fill="${FACE_COLORS[st]}" fill-opacity=".6"/>`);
    if (st === 'scan') p.push(`<rect class="scan-pulse" x="${X(f.x0) - 4}" y="${Y(f.y) - 9}" width="${(f.x1 - f.x0) * S + 8}" height="18" rx="6"/>`);
    if (!compact) p.push(`<text class="map-label" x="${X(f.x0)}" y="${Y(f.y) - 8}">${rackLabel(shelf)}</text>`);
  }

  // Tags RFID (allumes quand un robot vient de les lire)
  const tpos = tagPositions();
  if (layers.tags) {
    for (const [tag, xy] of Object.entries(tpos)) {
      const age = STATE.tags[tag].last_read_age;
      const fresh = age != null && age < FRESH_TAG_SEC;
      p.push(`<circle cx="${X(xy[0])}" cy="${Y(xy[1])}" r="${fresh ? 3.4 : compact ? 2 : 2.4}" fill="${fresh ? '#ffd29e' : '#8a8c93'}"${fresh ? ' style="filter:drop-shadow(0 0 3px #ff8a3d)"' : ''}><title>${tag} · ${rackLabel(STATE.tags[tag].shelf)} · lu il y a ${fmtAge(age)}</title></circle>`);
    }
  }

  // Cibles des missions en cours
  for (const t of STATE.tasks) {
    if (['succeeded', 'failed', 'rejected', 'canceled'].includes(t.state)) continue;
    const n = STATIC.nodes[t.target]; if (!n) continue;
    const c = t.type === 'rfid_scan' ? '#818cf8' : '#7c8cff';
    p.push(`<circle cx="${X(n.x)}" cy="${Y(n.y)}" r="7" fill="none" stroke="${c}" stroke-width="1.6" stroke-dasharray="${t.state === 'auction' ? '2 3' : ''}"/>`);
  }

  // Robots, trajectoires, lectures en cours
  for (const [r, d] of Object.entries(STATE.robots)) {
    if (d.x == null) continue;
    const c = robotColor(r);
    if (layers.trails && trails[r]?.length > 1) {
      p.push(`<polyline points="${trails[r].map(([x, y]) => `${X(x).toFixed(1)},${Y(y).toFixed(1)}`).join(' ')}" fill="none" stroke="${c}" stroke-opacity=".35" stroke-width="1.5"/>`);
    }
    if (layers.beams && d.reads?.length) {
      for (const tag of d.reads) {
        const xy = tpos[tag]; if (!xy) continue;
        p.push(`<line x1="${X(d.x)}" y1="${Y(d.y)}" x2="${X(xy[0])}" y2="${Y(xy[1])}" stroke="${c}" stroke-opacity=".4" stroke-width="1"/>`);
      }
    }
    if (d.task) {
      const n = STATIC.nodes[d.task.target];
      if (n) p.push(`<line x1="${X(d.x)}" y1="${Y(d.y)}" x2="${X(n.x)}" y2="${Y(n.y)}" stroke="${c}" stroke-opacity=".5" stroke-dasharray="5 4"/>`);
    }
    const pts = FOOTPRINT.map(([fx, fy]) => `${(fx * S).toFixed(1)},${(-fy * S).toFixed(1)}`).join(' ');
    p.push(`<g transform="translate(${X(d.x)},${Y(d.y)}) rotate(${deg(d.yaw)})">
      <polygon points="${pts}" fill="${c}" fill-opacity="${d.online ? .85 : .25}" stroke="#fff" stroke-opacity=".7"/>
      <polygon points="${0.46 * S},0 ${0.2 * S},${0.12 * S} ${0.2 * S},${-0.12 * S}" fill="#111111"/></g>`);
    p.push(`<text class="robot-label" x="${X(d.x) + 12}" y="${Y(d.y) - 10}">${r.replace('robot', 'R')}</text>`);
  }
  p.push('</svg>');
  return p.join('');
}

// ------------------------------------------------------------------ panels
function kpi(label, value, sub, cls = '') {
  return `<div class="card kpi ${cls}"><small>${label}</small><strong>${value}</strong><span>${sub}</span></div>`;
}

function rfidKpis() {
  const f = STATE.rfid; const c = STATE.task_counts;
  const recent = Object.values(f.shelves).filter((s) => s.last_read_age != null && s.last_read_age < RECENT_SCAN_SEC).length;
  return [
    kpi('Lectures RFID', f.total_reads, `${f.reads_per_s.toFixed(1)} lectures / s`),
    kpi('Tags lus', `${f.tags_seen}/${f.tags_total}`, 'tags vus au moins une fois', f.tags_seen === f.tags_total && f.tags_total ? 'good' : ''),
    kpi('Racks scannes', `${recent}/${Object.keys(f.shelves).length}`, `dans les ${RECENT_SCAN_SEC} dernieres s`),
    kpi('Scans RFID', `${c.scans_done}/${c.scans_total}`, 'missions de scan terminees'),
  ];
}

function renderOverview() {
  const c = STATE.task_counts;
  const online = Object.values(STATE.robots).filter((r) => r.online).length;
  setHTML($('kpis'), [
    kpi('Robots en ligne', `${online}/${STATIC.robots.length}`, 'flotte', online === STATIC.robots.length ? 'good' : 'warn'),
    kpi('Missions reussies', c.succeeded, `${c.failed} echouees · ${c.open} ouvertes`, 'good'),
    ...rfidKpis(),
  ].join(''));
  setHTML($('ov-map'), renderMap(true));
  $('ov-map-meta').textContent = `${STATIC.world} · ${Object.keys(STATIC.faces).length} racks RFID`;
  setHTML($('ov-chart'), renderChart());
  setHTML($('ov-fleet'), renderFleet());
  $('fleet-meta').textContent = `${online} en ligne`;
  setHTML($('ov-activity'), renderFeed(STATE.activity.slice(0, 18)));
}

function renderChart() {
  const h = STATE.rfid.history;
  if (h.length < 2) return '<p class="empty">En attente des premieres lectures RFID…</p>';
  const W = 520, H = 190, padL = 34, padB = 20;
  const t0 = h[0][0], t1 = h[h.length - 1][0] || t0 + 1;
  const ymax = Math.max(1, ...h.map((r) => r[1])) * 1.15;
  const px = (t) => padL + ((t - t0) / Math.max(1, t1 - t0)) * (W - padL - 6);
  const py = (v) => 6 + (1 - v / ymax) * (H - padB - 6);
  const line = h.map((r) => `${px(r[0]).toFixed(1)},${py(r[1]).toFixed(1)}`).join(' ');
  const area = `${px(t0).toFixed(1)},${py(0)} ${line} ${px(t1).toFixed(1)},${py(0)}`;
  const ticks = [0, ymax / 2, ymax].map((v) => `<text x="2" y="${py(v) + 3}" fill="#9a9ca3" font-size="9">${v.toFixed(0)}/s</text><line x1="${padL}" x2="${W}" y1="${py(v)}" y2="${py(v)}" stroke="rgba(255,255,255,.08)"/>`).join('');
  return `<svg viewBox="0 0 ${W} ${H}">${ticks}
    <polygon points="${area}" fill="rgba(255,138,61,.12)"/>
    <polyline points="${line}" fill="none" stroke="#ff8a3d" stroke-width="2.2"/>
    <text x="${W - 4}" y="${H - 4}" fill="#9a9ca3" font-size="9" text-anchor="end">${((t1 - t0) / 60).toFixed(1)} min</text></svg>`;
}

function renderFleet() {
  return STATIC.robots.map((r) => {
    const d = STATE.robots[r] || {};
    const status = !d.online ? '<span class="badge b-fail">hors ligne</span>'
      : d.task ? `<span class="badge ${d.task.type === 'rfid_scan' ? 'b-scan' : 'b-pick'}">${d.task.type === 'rfid_scan' ? 'scan' : 'picking'} ${rackLabel(d.task.target)}</span>`
        : d.nav_busy ? '<span class="badge b-run">exploration</span>' : '<span class="badge b-idle">libre</span>';
    const pos = d.x == null ? 'position inconnue' : `(${d.x.toFixed(1)}, ${d.y.toFixed(1)}) · ${d.reads?.length || 0} tags lus`;
    return `<div class="robot-card"><span class="dot" style="background:${robotColor(r)}"></span><div><strong>${r}</strong><small>${pos}</small></div>${status}</div>`;
  }).join('');
}

function renderFeed(events) {
  if (!events.length) return '<li class="empty">Aucune activite pour le moment</li>';
  return events.map((e) => `<li class="k-${e.kind}"><i></i><span>${esc(e.text)}</span><time>${ago(e.t)}</time></li>`).join('');
}

function renderInspector() {
  if (!selected) {
    return `<h2>Inspecteur</h2><p class="muted">Cliquez sur un rack de la carte pour voir ses tags RFID, quand ils ont ete lus et par quel robot, et pour demander un scan.</p>
      <div class="stat-grid" style="grid-template-columns:1fr 1fr">
      <div><small>Robots</small><strong>${STATIC.robots.length}</strong></div>
      <div><small>Racks RFID</small><strong>${Object.keys(STATIC.faces).length}</strong></div></div>`;
  }
  const sh = STATE.rfid.shelves[selected] || {};
  const tags = Object.entries(STATE.tags).filter(([, d]) => d.shelf === selected).sort(([a], [b]) => a.localeCompare(b));
  const rows = tags.map(([t, d]) => `<tr><td>${t}</td><td>${fmtAge(d.last_read_age)}</td>
      <td>${d.robot ? `<span style="color:${robotColor(d.robot)}">${d.robot}</span>` : '—'}</td></tr>`).join('');
  return `<p class="eyebrow">Rack RFID</p><h2>${rackLabel(selected)} <span class="muted">${selected}</span></h2>
    <p class="muted">Dernier scan : ${sh.last_read_age == null ? 'jamais' : `il y a ${fmtAge(sh.last_read_age)}${sh.last_robot ? ` par ${sh.last_robot}` : ''}`}</p>
    <div class="stat-grid">
      <div><small>Tags</small><strong>${sh.tags ?? 0}</strong></div>
      <div><small>Lus (60 s)</small><strong>${sh.read_recent ?? 0}</strong></div>
      <div><small>Lectures</small><strong>${sh.reads ?? 0}</strong></div>
    </div>
    <div class="btn-row">
      <button class="btn btn-primary" data-action="scan">Demander un scan RFID</button>
      <button class="btn" data-action="pick">Creer un picking</button>
    </div>
    <table class="table"><tr><th>Tag</th><th>Derniere lecture</th><th>Robot</th></tr>${rows || '<tr><td colspan="3" class="empty">Aucun tag</td></tr>'}</table>`;
}

function renderMissions() {
  const c = STATE.task_counts;
  setHTML($('mission-kpis'), [
    kpi('Taches ouvertes', c.open, 'en enchere ou en cours', 'warn'),
    kpi('Reussies', c.succeeded, 'missions terminees', 'good'),
    kpi('Echouees', c.failed, 'apres nouvel essai', c.failed ? 'bad' : ''),
    kpi('Scans RFID', `${c.scans_done}/${c.scans_total}`, 'missions de scan'),
  ].join(''));
  const rows = STATE.tasks.map((t) => {
    const st = t.state === 'succeeded' ? 'b-ok' : ['failed', 'rejected', 'canceled'].includes(t.state) ? 'b-fail' : t.state === 'auction' ? 'b-idle' : 'b-run';
    const claims = Object.keys(t.claims || {}).length;
    return `<tr><td>${esc(t.id.replace('task_', ''))}</td><td><span class="badge ${t.type === 'rfid_scan' ? 'b-scan' : 'b-pick'}">${t.type === 'rfid_scan' ? 'scan RFID' : 'picking'}</span></td>
      <td>${rackLabel(t.target)}</td><td>${t.robot ? `<span style="color:${robotColor(t.robot)}">${t.robot}</span>` : '—'}</td>
      <td><span class="badge ${st}">${t.state}</span></td><td>${t.elapsed != null ? `${t.elapsed} s` : '—'}</td>
      <td>${claims > 1 ? `<span class="badge b-run">${claims} conflit</span>` : claims ? '1' : '—'}</td></tr>`;
  }).join('');
  setHTML($('task-table'), `<tr><th>Tache</th><th>Type</th><th>Cible</th><th>Robot</th><th>Etat</th><th>Duree</th><th>Revend.</th></tr>${rows}`);
  setHTML($('consensus-feed'), renderFeed(STATE.activity.filter((e) => ['consensus', 'task', 'done', 'fail', 'scan'].includes(e.kind))));
}

function renderReads() {
  setHTML($('reads-kpis'), rfidKpis().join(''));
  const rows = Object.keys(STATIC.faces).sort().map((s) => {
    const d = STATE.rfid.shelves[s] || {};
    const frac = d.tags ? (d.read_recent || 0) / d.tags : 0;
    return `<tr class="clickable" data-shelf="${s}"><td><strong>${rackLabel(s)}</strong> <span class="muted">${s}</span></td><td>${d.tags ?? 0}</td>
      <td style="min-width:110px"><div class="bar"><i style="width:${(frac * 100).toFixed(0)}%;background:#4ade80"></i></div><small class="muted">${d.read_recent ?? 0} lus (60 s)</small></td>
      <td>${d.reads ?? 0}</td><td>${fmtAge(d.last_read_age)}</td>
      <td>${d.last_robot ? `<span style="color:${robotColor(d.last_robot)}">${d.last_robot}</span>` : '—'}</td></tr>`;
  }).join('');
  setHTML($('rack-table'), `<tr><th>Rack</th><th>Tags</th><th>Lus recemment</th><th>Lectures</th><th>Dernier scan</th><th>Par</th></tr>${rows}`);
  const feed = STATE.rfid.recent.map((e) => `<li class="k-scan"><i style="background:${robotColor(e.robot)}"></i>
      <span><strong style="color:${robotColor(e.robot)}">${e.robot}</strong> a lu ${e.count} tag${e.count > 1 ? 's' : ''}${e.shelves.length ? ` sur ${e.shelves.map(rackLabel).join(', ')}` : ''}
      <span class="muted">${e.tags.join(' ')}${e.count > e.tags.length ? ' …' : ''}</span></span><time>${ago(e.t)}</time></li>`).join('');
  setHTML($('reads-feed'), feed || '<li class="empty">Aucune lecture pour le moment</li>');
}

// ------------------------------------------------------------------ main loop
function render() {
  if (!STATE || !STATIC) return;
  for (const [r, d] of Object.entries(STATE.robots)) {
    if (d.x == null) continue;
    const t = (trails[r] ||= []);
    const last = t[t.length - 1];
    if (!last || Math.hypot(last[0] - d.x, last[1] - d.y) > 0.15) { t.push([d.x, d.y]); if (t.length > 160) t.shift(); }
  }
  $('sim-clock').textContent = fmtClock(STATE.sim_elapsed);
  if (view === 'overview') renderOverview();
  if (view === 'map') { setHTML($('main-map'), renderMap(false)); setHTML($('inspector'), renderInspector()); }
  if (view === 'missions') renderMissions();
  if (view === 'reads') renderReads();
}

function setView(v) {
  view = v;
  document.querySelectorAll('.nav-item').forEach((b) => b.classList.toggle('active', b.dataset.view === v));
  document.querySelectorAll('.view').forEach((s) => s.classList.toggle('active', s.id === `view-${v}`));
  $('view-eyebrow').textContent = TITLES[v][0]; $('view-title').textContent = TITLES[v][1];
  if (location.hash !== `#${v}`) history.replaceState(null, '', `#${v}`);
  render();
}

function selectShelf(shelf) { selected = shelf; if (view !== 'map') setView('map'); else render(); }

function wire() {
  document.querySelectorAll('.nav-item').forEach((b) => b.addEventListener('click', () => setView(b.dataset.view)));
  document.querySelectorAll('#map-toggles input').forEach((i) => i.addEventListener('change', () => { layers[i.dataset.layer] = i.checked; render(); }));
  for (const id of ['ov-map', 'main-map']) {
    $(id).addEventListener('pointerdown', (e) => { const s = e.target.closest('.face')?.dataset.shelf; if (s) selectShelf(s); });
  }
  $('rack-table').addEventListener('click', (e) => { const s = e.target.closest('tr[data-shelf]')?.dataset.shelf; if (s) selectShelf(s); });
  $('inspector').addEventListener('click', async (e) => {
    const a = e.target.closest('[data-action]')?.dataset.action;
    if (!a || !selected) return;
    if (a === 'scan') toast(await post('/api/scan', { shelf: selected }) ? `Scan RFID demande sur ${rackLabel(selected)}` : 'Echec de la demande');
    if (a === 'pick') toast(await post('/api/pick', { shelf: selected, priority: 1.5 }) ? `Tache de picking creee vers ${rackLabel(selected)}` : 'Echec de la creation');
  });
}

function connect() {
  const es = new EventSource('/api/stream');
  const conn = $('conn');
  es.onopen = () => { conn.className = 'pill pill-on'; conn.lastElementChild.textContent = 'Connecte a ROS 2'; };
  es.onerror = () => { conn.className = 'pill pill-off'; conn.lastElementChild.textContent = 'Deconnecte'; };
  es.onmessage = (ev) => { STATE = JSON.parse(ev.data); render(); };
}

(async function init() {
  wire();
  for (;;) {
    try { STATIC = await (await fetch('/api/static')).json(); break; } catch { await new Promise((r) => setTimeout(r, 1500)); }
  }
  computeBounds();
  $('world-name').textContent = `Monde : ${STATIC.world}`;
  const [hv, hs] = location.hash.slice(1).split('/');
  if (TITLES[hv]) setView(hv);
  if (hs && STATIC.faces[hs]) selected = hs;
  connect();
})();
