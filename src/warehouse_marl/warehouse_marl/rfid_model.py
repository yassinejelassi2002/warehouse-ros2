"""
rfid_model.py
-------------
Couche RFID du sujet ("RFID-based sensing ... adaptive decision-making under
uncertain conditions"), sans dependance ROS2 pour pouvoir etre testee seule.

Trois parties :

1. ``RFIDReaderModel`` : modele physique d'un lecteur UHF embarque.
   - probabilite de lecture decroissante avec la distance (logistique),
     nulle au-dela de la portee maximale -> lectures manquees ;
   - RSSI selon un modele de perte en espace libre (log-distance) avec bruit
     gaussien -> l'estimation de distance a partir du RSSI est imprecise ;
   - lectures fantomes (tags lus alors qu'ils sont hors portee nominale).

2. ``InventoryWorld`` : verite terrain simulee. Des articles portant un tag
   RFID sont ranges sur les faces des racks. Le WMS (Warehouse Management
   System) connait l'etat initial, mais le monde derive sans que le WMS soit
   mis a jour : articles deplaces sur un autre rack, articles retires,
   arrivees non enregistrees.

3. ``InventoryBelief`` : estimation probabiliste de l'inventaire. Pour chaque
   couple (article, rack), un log-odds de presence est mis a jour a chaque
   scan d'un robot (filtre de Bayes binaire, comme une grille d'occupation) :
   lecture -> evidence de presence, absence de lecture alors que le rack est
   a portee -> evidence d'absence, ponderee par la probabilite de lecture a
   cette distance. Le taux d'erreur d'inventaire est mesure contre la verite
   terrain, et compare a celui des seuls enregistrements WMS.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

# --------------------------------------------------------------------------- #
# Geometrie : faces des racks servies par chaque noeud "shelf" du graphe
# --------------------------------------------------------------------------- #

# Monde Gazebo rosnav_bot/worlds/warehouse.world : segments de rack de 2.8 m
# centres en x = 0, 3.5, 7 ; rangees de racks doubles en y = 8/7, 4.5/3.5,
# 1/0, -3/-4, -7/-8 (profondeur 0.5 m). Chaque ligne r du graphe (point de
# service dans une allee, cf. build_demo_warehouse) fait face a une face de
# rack. Les lignes 3 et 4 partagent la meme allee (y=-5.2) mais font face a
# deux racks opposes.
_RACK_FACE_Y = (4.75, 3.25, -2.75, -4.25, -6.75)
_SEGMENT_X = (0.0, 3.5, 7.0)
_SEGMENT_HALF_LENGTH = 1.2  # tags repartis sur 2.4 m de chaque segment


@dataclass(frozen=True)
class RackFace:
    shelf_id: str
    x0: float
    x1: float
    y: float

    def slot_points(self, n: int = 5) -> List[Tuple[float, float]]:
        """Points regulierement espaces le long de la face (positions
        possibles d'un article dont on ne connait pas l'emplacement exact)."""
        if n <= 1:
            return [((self.x0 + self.x1) / 2.0, self.y)]
        step = (self.x1 - self.x0) / (n - 1)
        return [(self.x0 + i * step, self.y) for i in range(n)]


def rack_faces(map_origin_x: float = 0.0,
               map_origin_y: float = 0.0) -> Dict[str, RackFace]:
    """Faces de rack dans le repere ``map`` (monde - origine de la carte)."""
    faces = {}
    for row, face_y in enumerate(_RACK_FACE_Y):
        for col, cx in enumerate(_SEGMENT_X):
            shelf_id = f'shelf_{row}_{col}'
            faces[shelf_id] = RackFace(
                shelf_id=shelf_id,
                x0=cx - _SEGMENT_HALF_LENGTH - map_origin_x,
                x1=cx + _SEGMENT_HALF_LENGTH - map_origin_x,
                y=face_y - map_origin_y,
            )
    return faces


# --------------------------------------------------------------------------- #
# Modele physique du lecteur
# --------------------------------------------------------------------------- #

@dataclass
class RFIDReaderModel:
    p_max: float = 0.95           # probabilite de lecture a courte distance
    range_50: float = 2.0         # distance (m) ou la probabilite vaut p_max/2
    falloff: float = 0.3          # raideur de la decroissance (m)
    max_range: float = 3.5        # au-dela : aucune lecture nominale
    rssi_at_1m: float = -45.0     # dBm
    path_loss_exponent: float = 2.0
    rssi_noise_db: float = 2.5
    ghost_rate: float = 0.01      # proba. par scan de lire un tag hors portee

    def read_probability(self, distance: float) -> float:
        if distance > self.max_range:
            return 0.0
        return self.p_max / (1.0 + math.exp((distance - self.range_50) / self.falloff))

    def rssi(self, distance: float, rng: random.Random) -> float:
        d = max(distance, 0.1)
        clean = self.rssi_at_1m - 10.0 * self.path_loss_exponent * math.log10(d)
        return clean + rng.gauss(0.0, self.rssi_noise_db)

    def range_from_rssi(self, rssi: float) -> float:
        return 10.0 ** ((self.rssi_at_1m - rssi) / (10.0 * self.path_loss_exponent))

    def face_read_probabilities(self, robot_xy: Tuple[float, float],
                                face: RackFace) -> Tuple[float, float]:
        """(moyenne, minimum) de la probabilite de lire un article present sur
        ``face`` dont la position exacte le long de la face est inconnue."""
        probs = [self.read_probability(math.dist(robot_xy, p)) for p in face.slot_points()]
        return sum(probs) / len(probs), min(probs)


# --------------------------------------------------------------------------- #
# Verite terrain (simulation du monde reel) + enregistrements WMS
# --------------------------------------------------------------------------- #

@dataclass
class Item:
    tag: str
    shelf: Optional[str]          # None = article sorti de l'entrepot
    slot_x: float                 # position le long de la face (repere map)


class InventoryWorld:
    """Etat reel de l'inventaire. Le WMS ne voit que l'etat initial et les
    operations enregistrees ; les evenements de ``drift`` ne sont pas
    enregistres (erreurs de rangement, demarque, arrivees non saisies)."""

    def __init__(self, faces: Dict[str, RackFace], items_per_shelf: int = 4,
                 seed: int = 7):
        self.faces = faces
        self.rng = random.Random(seed)
        self.items: Dict[str, Item] = {}
        self._next_id = 1
        for shelf_id in sorted(faces):
            for _ in range(items_per_shelf):
                self._new_item(shelf_id)
        self.wms: Dict[str, Optional[str]] = {
            tag: item.shelf for tag, item in self.items.items()}

    def _new_item(self, shelf_id: str) -> Item:
        tag = f'EPC-{self._next_id:04d}'
        self._next_id += 1
        face = self.faces[shelf_id]
        item = Item(tag=tag, shelf=shelf_id,
                    slot_x=self.rng.uniform(face.x0, face.x1))
        self.items[tag] = item
        return item

    def item_position(self, item: Item) -> Optional[Tuple[float, float]]:
        if item.shelf is None:
            return None
        return item.slot_x, self.faces[item.shelf].y

    def drift(self) -> str:
        """Un evenement non enregistre dans le WMS. Retourne sa description."""
        in_stock = [i for i in self.items.values() if i.shelf is not None]
        shelves = sorted(self.faces)
        event = self.rng.choices(
            ('misplaced', 'removed', 'unrecorded_arrival'), weights=(5, 3, 2))[0]
        if event == 'misplaced' and in_stock:
            item = self.rng.choice(in_stock)
            target = self.rng.choice([s for s in shelves if s != item.shelf])
            face = self.faces[target]
            src = item.shelf
            item.shelf = target
            item.slot_x = self.rng.uniform(face.x0, face.x1)
            return f'{item.tag} deplace {src} -> {target}'
        if event == 'removed' and in_stock:
            item = self.rng.choice(in_stock)
            src = item.shelf
            item.shelf = None
            return f'{item.tag} retire de {src}'
        item = self._new_item(self.rng.choice(shelves))
        return f'{item.tag} arrive (non enregistre) sur {item.shelf}'

    def truth(self) -> Dict[str, Optional[str]]:
        return {tag: item.shelf for tag, item in self.items.items()}

    def scan(self, robot_xy: Tuple[float, float], model: RFIDReaderModel,
             rng: random.Random) -> List[Tuple[str, float]]:
        """Lectures (tag, RSSI) d'un robot a ``robot_xy`` pour un cycle de scan."""
        reads = []
        for item in self.items.values():
            pos = self.item_position(item)
            if pos is None:
                continue
            d = math.dist(robot_xy, pos)
            if rng.random() < model.read_probability(d):
                reads.append((item.tag, model.rssi(d, rng)))
        if rng.random() < model.ghost_rate:
            far = [i for i in self.items.values() if i.shelf is not None]
            if far:
                item = rng.choice(far)
                d = math.dist(robot_xy, self.item_position(item))
                reads.append((item.tag, model.rssi(min(d, model.max_range), rng)))
        return reads


# --------------------------------------------------------------------------- #
# Croyance probabiliste sur l'inventaire
# --------------------------------------------------------------------------- #

class InventoryBelief:
    """Filtre de Bayes categoriel par article.

    Pour chaque article, une distribution de probabilite sur ses positions
    possibles : chaque rack x ``slots`` emplacements le long de sa face, plus
    l'hypothese "absent de l'entrepot". Les hypotheses s'excluent (un article
    n'est qu'a un seul endroit) : une evidence pour un rack est donc aussi
    une evidence contre les autres.

    Mise a jour a chaque scan d'un robot :
      - article lu avec un RSSI r : vraisemblance de chaque hypothese =
        P(lecture a cette distance) x densite N(r ; RSSI attendu, bruit),
        plus un terme de lecture fantome ;
      - article non lu : vraisemblance = 1 - P(lecture a cette distance).
    Les lectures successives depuis des positions differentes triangulent
    l'article, ce qui leve l'ambiguite entre segments voisins ou racks face
    a face. Un terme de transition (``relocation_rate``) permet a la
    croyance de suivre les articles deplaces ou retires.
    """

    def __init__(self, faces: Dict[str, RackFace], model: RFIDReaderModel,
                 wms: Dict[str, Optional[str]], prior_recorded: float = 0.9,
                 prior_absent: float = 0.05, slots: int = 5,
                 relocation_rate: float = 1.0 / 600.0,
                 min_observation: float = 0.5, rssi_span_db: float = 40.0):
        import numpy as np
        self._np = np
        self.faces = faces
        self.model = model
        self.shelves = sorted(faces)
        self.slots = slots
        self.prior_recorded = prior_recorded
        self.prior_absent = prior_absent
        self.relocation_rate = relocation_rate
        self.min_observation = min_observation
        self.ghost_density = model.ghost_rate / rssi_span_db
        points = []
        for shelf_id in self.shelves:
            points.extend(faces[shelf_id].slot_points(slots))
        self._points = np.array(points)                   # (H-1, 2)
        self.n_hyp = len(points) + 1                       # + absent
        self.tags: List[str] = []
        self._index: Dict[str, int] = {}
        self._belief = np.zeros((0, self.n_hyp))
        self._last_t: Optional[float] = None
        self.last_observed: Dict[str, float] = {s: -math.inf for s in self.shelves}
        self.reads = 0
        for tag, shelf in wms.items():
            self._ensure_item(tag, recorded_shelf=shelf)

    # -- helpers ----------------------------------------------------------
    def _prior_row(self, recorded_shelf: Optional[str]):
        np = self._np
        row = np.zeros(self.n_hyp)
        n_slots = self.n_hyp - 1
        if recorded_shelf in self.faces:
            k = self.shelves.index(recorded_shelf)
            row[:n_slots] = (1.0 - self.prior_recorded - self.prior_absent) / n_slots
            row[k * self.slots:(k + 1) * self.slots] += self.prior_recorded / self.slots
            row[-1] = self.prior_absent
        else:  # article inconnu du WMS : position totalement incertaine
            row[:] = 1.0 / self.n_hyp
        return row

    def _ensure_item(self, tag: str, recorded_shelf: Optional[str] = None) -> None:
        if tag in self._index:
            return
        self._index[tag] = len(self.tags)
        self.tags.append(tag)
        self._belief = self._np.vstack([self._belief, self._prior_row(recorded_shelf)])

    def _shelf_probs(self):
        """(articles x racks) probabilite que l'article soit sur le rack."""
        b = self._belief[:, :-1]
        return b.reshape(len(self.tags), len(self.shelves), self.slots).sum(axis=2)

    # -- update -----------------------------------------------------------
    def update(self, robot_xy: Tuple[float, float],
               reads: Iterable[Tuple[str, float]], now: float) -> Dict[str, int]:
        np = self._np
        reads = list(reads)
        for tag, _ in reads:
            self._ensure_item(tag)
        if not self.tags:
            return {}

        # Transition : un article peut avoir ete deplace depuis le dernier scan.
        if self._last_t is not None and now > self._last_t:
            eps = 1.0 - math.exp(-(now - self._last_t) * self.relocation_rate)
            self._belief = (1.0 - eps) * self._belief + eps / self.n_hyp
        self._last_t = now

        d = np.hypot(self._points[:, 0] - robot_xy[0], self._points[:, 1] - robot_xy[1])
        m = self.model
        p_read = np.where(d > m.max_range, 0.0,
                          m.p_max / (1.0 + np.exp((d - m.range_50) / m.falloff)))
        mu = m.rssi_at_1m - 10.0 * m.path_loss_exponent * np.log10(np.maximum(d, 0.1))

        miss = np.append(1.0 - p_read, 1.0)                     # absent : jamais lu
        self._belief *= miss                                    # tous les articles...
        counts: Dict[str, int] = {}
        for tag, rssi in reads:                                 # ...sauf ceux lus
            self.reads += 1
            i = self._index[tag]
            dens = np.exp(-0.5 * ((rssi - mu) / m.rssi_noise_db) ** 2) / (
                m.rssi_noise_db * math.sqrt(2.0 * math.pi))
            like = np.append(p_read * dens, 0.0) + self.ghost_density
            self._belief[i] = self._belief[i] / miss * like
        self._belief /= self._belief.sum(axis=1, keepdims=True)

        per_shelf = p_read.reshape(len(self.shelves), self.slots).mean(axis=1)
        for k, shelf_id in enumerate(self.shelves):
            if per_shelf[k] >= self.min_observation:
                self.last_observed[shelf_id] = now
                counts[shelf_id] = 0
        return counts

    # -- queries ----------------------------------------------------------
    def probability(self, tag: str, shelf_id: str) -> float:
        i, k = self._index[tag], self.shelves.index(shelf_id)
        return float(self._belief[i, k * self.slots:(k + 1) * self.slots].sum())

    def estimate(self) -> Dict[str, Optional[str]]:
        """Emplacement le plus probable de chaque article (None = absent)."""
        if not self.tags:
            return {}
        np = self._np
        shelf_p = self._shelf_probs()
        best = shelf_p.argmax(axis=1)
        absent = self._belief[:, -1]
        return {tag: (None if absent[i] >= shelf_p[i, best[i]] else self.shelves[best[i]])
                for i, tag in enumerate(self.tags)}

    def item_estimates(self) -> Dict[str, Tuple[Optional[str], float]]:
        """(emplacement le plus probable, sa probabilite) pour chaque article."""
        if not self.tags:
            return {}
        shelf_p = self._shelf_probs()
        best = shelf_p.argmax(axis=1)
        absent = self._belief[:, -1]
        result = {}
        for i, tag in enumerate(self.tags):
            p_best = float(shelf_p[i, best[i]])
            if absent[i] >= p_best:
                result[tag] = (None, float(absent[i]))
            else:
                result[tag] = (self.shelves[best[i]], p_best)
        return result

    def shelf_stock(self, threshold: float = 0.7) -> Dict[str, int]:
        shelf_p = self._shelf_probs()
        return {s: int((shelf_p[:, k] >= threshold).sum()) for k, s in enumerate(self.shelves)}

    def shelf_uncertainty(self, now: float, stale_after: float = 600.0) -> Dict[str, float]:
        """Score d'incertitude par rack = nombre d'articles dont la presence
        y est ambigue (0.2 < p < 0.8) + 3 x anciennete de la derniere lecture
        (0 = vient d'etre scanne, 1 = plus de ``stale_after`` s ou jamais)."""
        shelf_p = self._shelf_probs()
        scores = {}
        for k, shelf_id in enumerate(self.shelves):
            ambiguous = int(((shelf_p[:, k] > 0.2) & (shelf_p[:, k] < 0.8)).sum())
            age = now - self.last_observed[shelf_id]
            staleness = 1.0 if math.isinf(age) else min(age / stale_after, 1.0)
            scores[shelf_id] = ambiguous + 3.0 * staleness
        return scores


def inventory_error_rate(estimate: Dict[str, Optional[str]],
                         truth: Dict[str, Optional[str]]) -> float:
    """Part des articles (connus de la verite ou de l'estimation) dont
    l'emplacement estime differe de l'emplacement reel. Un article sorti de
    l'entrepot (None) doit etre estime absent."""
    tags = set(truth) | set(estimate)
    if not tags:
        return 0.0
    wrong = sum(1 for t in tags if estimate.get(t) != truth.get(t))
    return wrong / len(tags)


def inventory_error_breakdown(estimate: Dict[str, Optional[str]],
                              truth: Dict[str, Optional[str]]) -> Dict[str, int]:
    """Erreurs par type : article present mais estime absent (missed), absent
    mais estime present (phantom), present sur un autre rack (wrong_shelf)."""
    counts = {'missed': 0, 'phantom': 0, 'wrong_shelf': 0}
    for tag in set(truth) | set(estimate):
        real, est = truth.get(tag), estimate.get(tag)
        if real == est:
            continue
        if est is None:
            counts['missed'] += 1
        elif real is None:
            counts['phantom'] += 1
        else:
            counts['wrong_shelf'] += 1
    return counts
