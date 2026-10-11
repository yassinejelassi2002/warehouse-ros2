"""
rfid_experiment.py
-------------------
Outils d'evaluation de la couche RFID, sans ROS 2 ni Gazebo.

``rfid_offline_experiment`` : simulation rapide (quelques secondes) avec le
meme modele que les noeuds ROS (rfid_model). Des robots visitent des racks
pendant que l'inventaire derive ; on compare trois strategies :
  - wms_only    : aucune mise a jour, enregistrements WMS seuls ;
  - rfid_random : croyance RFID, robots visitant des racks au hasard
                  (lectures opportunistes pendant le picking) ;
  - rfid_adaptive : croyance RFID + une visite sur trois dirigee vers le
                  rack le plus incertain (missions de scan, comme
                  inventory_node -> /rfid/scan_requests).
Moyenne sur plusieurs graines ; ecrit un CSV et une figure.

    ros2 run warehouse_marl rfid_offline_experiment --minutes 60 --seeds 10

``plot_rfid_metrics`` : trace un CSV produit par rfid_metrics_logger
(simulation Gazebo complete).

    ros2 run warehouse_marl plot_rfid_metrics rfid_metrics.csv
"""

import argparse
import csv
import random
import statistics

from warehouse_marl.rfid_model import (
    InventoryBelief, InventoryWorld, RFIDReaderModel, inventory_error_rate, rack_faces)
from warehouse_marl.warehouse_graph import build_demo_warehouse

STRATEGIES = ('wms_only', 'rfid_random', 'rfid_adaptive')


def run_once(strategy, seed, minutes, robots, drift_per_minute, scans_per_visit,
             demand_skew=0.0):
    ox, oy = -1.5, -1.0
    faces = rack_faces(ox, oy)
    graph = build_demo_warehouse(ox, oy)
    model = RFIDReaderModel()
    world = InventoryWorld(faces, seed=seed)
    rng = random.Random(seed + 1000)
    belief = InventoryBelief(faces, model, world.wms)
    shelves = sorted(faces)
    # Demande de type Zipf (classes ABC) : quelques racks tres sollicites,
    # les autres rarement visites. demand_skew=0 -> demande uniforme.
    popularity = shelves[:]
    random.Random(seed + 2000).shuffle(popularity)
    weights = [1.0 / (rank + 1) ** demand_skew for rank in range(len(popularity))]
    t = 0.0
    series = []
    for minute in range(1, minutes + 1):
        for _ in range(drift_per_minute):
            world.drift()
        if strategy != 'wms_only':
            for visit in range(robots):
                if strategy == 'rfid_adaptive' and visit % 3 == 0:
                    scores = belief.shelf_uncertainty(t)
                    shelf = max(shelves, key=lambda s: scores[s])
                else:
                    shelf = rng.choices(popularity, weights=weights)[0]
                # The robot drives along the shelf's aisle from a random
                # point to the service point, scanning once per step (as in
                # Gazebo, where reads happen while driving).
                node = graph.G.nodes[shelf]
                x0 = rng.uniform(-1.5, 9.5) - (-1.5)
                for k in range(scans_per_visit + 1):
                    x = x0 + (node['x'] - x0) * k / scans_per_visit
                    xy = (x, node['y'] + rng.uniform(-0.15, 0.15))
                    belief.update(xy, world.scan(xy, model, rng), t)
                    t += 1.0
        t = max(t, minute * 60.0)
        truth = world.truth()
        estimate = world.wms if strategy == 'wms_only' else belief.estimate()
        series.append(inventory_error_rate(estimate, truth))
    return series


def offline_main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--minutes', type=int, default=60)
    parser.add_argument('--seeds', type=int, default=10)
    parser.add_argument('--robots', type=int, default=3, help='visites de racks par minute')
    parser.add_argument('--drift-per-minute', type=int, default=2)
    parser.add_argument('--scans-per-visit', type=int, default=12)
    parser.add_argument('--demand-skew', type=float, default=1.2,
                        help='exposant Zipf de la demande par rack (0 = uniforme)')
    parser.add_argument('--out', default='rfid_offline_experiment')
    args = parser.parse_args(argv)

    results = {}
    for strategy in STRATEGIES:
        runs = [run_once(strategy, seed, args.minutes, args.robots,
                         args.drift_per_minute, args.scans_per_visit, args.demand_skew)
                for seed in range(args.seeds)]
        results[strategy] = [statistics.mean(r[m] for r in runs) for m in range(args.minutes)]

    csv_path = f'{args.out}.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['minute', *STRATEGIES])
        for m in range(args.minutes):
            writer.writerow([m + 1, *(round(results[s][m], 4) for s in STRATEGIES)])

    print(f'Taux d erreur d inventaire moyen ({args.seeds} graines, '
          f'{args.drift_per_minute} evenements non enregistres / min, '
          f'demande Zipf {args.demand_skew}) :')
    last = slice(args.minutes // 2, args.minutes)
    for s in STRATEGIES:
        print(f'  {s:14s} fin = {results[s][-1]:.3f}   moyenne 2e moitie = '
              f'{statistics.mean(results[s][last]):.3f}')
    print(f'CSV : {csv_path}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib absent : pas de figure.')
        return
    labels = {'wms_only': 'WMS seul', 'rfid_random': 'RFID (lectures opportunistes)',
              'rfid_adaptive': 'RFID + scans adaptatifs'}
    fig, ax = plt.subplots(figsize=(7, 4))
    for s in STRATEGIES:
        ax.plot(range(1, args.minutes + 1), [100 * v for v in results[s]], label=labels[s])
    ax.set_xlabel('Temps (min)')
    ax.set_ylabel("Taux d'erreur d'inventaire (%)")
    ax.set_ylim(0, 100)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(f'{args.out}.png', dpi=150)
    print(f'Figure : {args.out}.png')


def plot_main(argv=None):
    parser = argparse.ArgumentParser(description='Trace un CSV de rfid_metrics_logger.')
    parser.add_argument('csv')
    parser.add_argument('--out', default=None)
    args = parser.parse_args(argv)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print('CSV vide.')
        return
    t0 = float(rows[0]['stamp'])
    t = [(float(r['stamp']) - t0) / 60.0 for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    ax1.plot(t, [100 * float(r['rfid_error_rate']) for r in rows], label='Croyance RFID')
    ax1.plot(t, [100 * float(r['wms_error_rate']) for r in rows], label='WMS seul')
    ax1.set_ylabel("Erreur d'inventaire (%)")
    ax1.set_ylim(0, 100)
    ax1.grid(alpha=0.3)
    ax1.legend()
    ax2.plot(t, [int(r['pick_tasks']) for r in rows], label='Taches de picking')
    ax2.plot(t, [int(r['pick_errors']) for r in rows], label='Picking vers rack vide')
    ax2.plot(t, [int(r['scan_tasks']) for r in rows], label='Missions de scan RFID')
    ax2.set_xlabel('Temps simule (min)')
    ax2.set_ylabel('Cumul')
    ax2.grid(alpha=0.3)
    ax2.legend()
    fig.tight_layout()
    out = args.out or args.csv.rsplit('.', 1)[0] + '.png'
    fig.savefig(out, dpi=150)
    print(f'Figure : {out}')
