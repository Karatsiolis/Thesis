"""
sumo_rasmha.py
==============
SUMO + RASMHA integration για τη διπλωματική.

Αντικαθιστά το χειροκίνητο __main__ του thesis.py με πραγματική
προσομοίωση SUMO: τα bs_time_windows συλλέγονται από την κίνηση
των οχημάτων στον χάρτη αντί να ορίζονται χειροκίνητα.

Δομή:
  Phase 1 — SUMO: τα οχήματα κινούνται, καταγράφονται τα V2I windows
  Phase 2 — RASMHA: εκτελείται ο αλγόριθμος με τα πραγματικά windows

Απαιτεί:
  - thesis.py στον ίδιο φάκελο
  - SUMO εγκατεστημένο και SUMO_HOME ορισμένο
  - map.net.xml + map.sumocfg μέσα στον φάκελο Thesis/
"""

from __future__ import annotations

import os
import sys
import math
from pathlib import Path
from typing import Dict, Tuple

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    raise EnvironmentError("SUMO_HOME is not set")

import traci
import sumolib

from thesis import (
    Task,
    SubTask,
    MECServer,
    mobility_aware_task_offloading_main,
)


# ============================================================
# Scenario configuration
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

SUMOCFG = str(BASE_DIR / "map.sumocfg")
NET_FILE = str(BASE_DIR / "map.net.xml")

# Αν τα map.sumocfg και map.net.xml είναι στον ίδιο φάκελο με το αρχείο αυτό,
# τότε χρησιμοποίησε αντί για τα παραπάνω:
# SUMOCFG = str(BASE_DIR / "map.sumocfg")
# NET_FILE = str(BASE_DIR / "map.net.xml")


# ── AV routes (lon, lat WGS84) ────────────────────────────
AV1_START = (21.788317, 40.302653)
AV1_END   = (21.797086, 40.298967)

AV2_START = (21.790000, 40.301000)
AV2_END   = (21.796000, 40.299500)


# ── BS positions (lon, lat) ───────────────────────────────
BS_COORDS: Dict[int, Tuple[float, float]] = {
    1: (21.7900, 40.3020),
    2: (21.7930, 40.3008),
    3: (21.7958, 40.2993),
}

BS_COVERAGE_M = 150.0
V2I_RATE = 2.0e7   # bps


# ── MEC servers ───────────────────────────────────────────
SERVERS: Dict[int, MECServer] = {
    1: MECServer(bs_id=1, F_total=40e9, C_max=10),
    2: MECServer(bs_id=2, F_total=40e9, C_max=10),
    3: MECServer(bs_id=3, F_total=40e9, C_max=10),
}


# ============================================================
# Helpers
# ============================================================

def _dist(p1, p2) -> float:
    """Euclidean distance σε SUMO x/y συντεταγμένες, δηλαδή σε μέτρα."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def _find_edge(net, lon: float, lat: float, radius: float = 30):
    """
    Βρίσκει το κοντινότερο drivable edge κοντά στο δοσμένο lon/lat.
    Χρησιμοποιείται για να τοποθετηθούν τα AVs πάνω στο οδικό δίκτυο.
    """
    x, y = net.convertLonLat2XY(lon, lat)
    r = radius

    while r <= 500:
        neighbors = net.getNeighboringEdges(x, y, r)

        drivable = [
            (edge, dist) for edge, dist in neighbors
            if not edge.isSpecial() and edge.allows("passenger")
        ]

        if drivable:
            edge, _ = min(drivable, key=lambda item: item[1])
            return edge

        r *= 2

    raise RuntimeError(f"Δεν βρέθηκε drivable edge κοντά στο ({lat}, {lon})")


# ============================================================
# Phase 1 — SUMO
# ============================================================

def run_sumo() -> Dict[int, Dict[int, Tuple[float, float]]]:
    """
    Εκκινεί SUMO-GUI με AV 1 και AV 2.

    Επιστρέφει:
        windows[av_id][bs_id] = (t_enter, t_exit)

    Παράδειγμα:
        windows[1][3] = (53.0, 61.0)
        σημαίνει ότι το AV1 ήταν μέσα στην κάλυψη του BS3 από 53s έως 61s.
    """

    net = sumolib.net.readNet(NET_FILE)

    # Μετατροπή BS lon/lat σε SUMO x/y
    bs_xy = {
        bs_id: net.convertLonLat2XY(lon, lat)
        for bs_id, (lon, lat) in BS_COORDS.items()
    }

    # Εύρεση αρχικού και τελικού edge για κάθε AV
    av_edges = {
        1: (_find_edge(net, *AV1_START), _find_edge(net, *AV1_END)),
        2: (_find_edge(net, *AV2_START), _find_edge(net, *AV2_END)),
    }

    windows: Dict[int, Dict[int, Tuple[float, float]]] = {
        av_id: {} for av_id in av_edges
    }

    in_range: Dict[int, Dict[int, float | None]] = {
        av_id: {bs_id: None for bs_id in BS_COORDS}
        for av_id in av_edges
    }

    traci.start(["sumo-gui", "-c", SUMOCFG])

    try:
        # --------------------------------------------------------
        # Προσθήκη BSs ως POIs στο SUMO-GUI
        # --------------------------------------------------------
        for bs_id, (x, y) in bs_xy.items():
            traci.poi.add(
                f"BS{bs_id}",
                x,
                y,
                color=(255, 0, 0, 220),
                layer=10,
                width=10,
                height=10,
            )

        # --------------------------------------------------------
        # Εισαγωγή οχημάτων
        # --------------------------------------------------------
        for av_id, (start_edge, end_edge) in av_edges.items():
            stage = traci.simulation.findRoute(
                start_edge.getID(),
                end_edge.getID()
            )

            if not stage.edges:
                raise RuntimeError(f"Δεν βρέθηκε διαδρομή για AV {av_id}")

            route_id = f"route_av{av_id}"
            veh_id = f"av{av_id}"

            traci.route.add(route_id, stage.edges)

            traci.vehicle.add(
                veh_id,
                route_id,
                depart="now",
                departLane="best",
                departSpeed="0",
            )

            print(
                f"AV {av_id} inserted | "
                f"length={stage.length:.0f} m | "
                f"est. travel={stage.travelTime:.1f} s"
            )

        # --------------------------------------------------------
        # Simulation loop μόνο για τα δικά μας AVs
        # --------------------------------------------------------
        target_vehicles = {f"av{av_id}" for av_id in av_edges}
        MAX_SIM_TIME = 120.0

        while target_vehicles and traci.simulation.getTime() <= MAX_SIM_TIME:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # Έλεγχος αν κάποιο AV έφτασε στον προορισμό
            arrived = set(traci.simulation.getArrivedIDList())

for veh_id in list(target_vehicles):
    if veh_id in arrived:
        target_vehicles.remove(veh_id)

        # Παίρνουμε το av_id από το όνομα "av1", "av2", ...
        av_id = int(veh_id.replace("av", ""))

        print(f"  t={t:.1f}s  {veh_id} έφτασε στον προορισμό")

        # Κλείνουμε αμέσως όσα BS windows ήταν ανοιχτά για αυτό το AV
        for bs_id, t_enter in in_range[av_id].items():
            if t_enter is not None:
                windows[av_id][bs_id] = (t_enter, t)
                in_range[av_id][bs_id] = None

                print(
                    f"  t={t:.1f}s  AV{av_id} → τέλος διαδρομής μέσα στο BS{bs_id} "
                    f"window={windows[av_id][bs_id]}"
                )

            # Έλεγχος κάλυψης για κάθε AV και κάθε BS
            for av_id in av_edges:
                veh_id = f"av{av_id}"

                if veh_id not in traci.vehicle.getIDList():
                    continue

                pos = traci.vehicle.getPosition(veh_id)

                for bs_id, bs_pos in bs_xy.items():
                    in_cov = _dist(pos, bs_pos) <= BS_COVERAGE_M

                    # Το AV μόλις μπήκε στην κάλυψη του BS
                    if in_cov and in_range[av_id][bs_id] is None:
                        in_range[av_id][bs_id] = t
                        print(f"  t={t:.1f}s  AV{av_id} → εντός BS{bs_id}")

                    # Το AV μόλις βγήκε από την κάλυψη του BS
                    elif not in_cov and in_range[av_id][bs_id] is not None:
                        t_enter = in_range[av_id][bs_id]
                        windows[av_id][bs_id] = (t_enter, t)
                        in_range[av_id][bs_id] = None

                        print(
                            f"  t={t:.1f}s  AV{av_id} → εκτός  BS{bs_id} "
                            f"window={windows[av_id][bs_id]}"
                        )

        # --------------------------------------------------------
        # Κλείσιμο windows για όσα AV τελείωσαν ενώ ήταν ακόμα μέσα σε BS
        # --------------------------------------------------------
        t_end = traci.simulation.getTime()

        for av_id in av_edges:
            for bs_id, t_enter in in_range[av_id].items():
                if t_enter is not None:
                    windows[av_id][bs_id] = (t_enter, t_end)
                    in_range[av_id][bs_id] = None

                    print(
                        f"  t={t_end:.1f}s  AV{av_id} → τέλος προσομοίωσης μέσα στο BS{bs_id} "
                        f"window={windows[av_id][bs_id]}"
                    )

        if traci.simulation.getTime() > MAX_SIM_TIME:
            print(
                f"Προειδοποίηση: η προσομοίωση σταμάτησε στο MAX_SIM_TIME={MAX_SIM_TIME}s."
            )

    finally:
        traci.close()

    return windows


# ============================================================
# Phase 2 — Tasks και RASMHA
# ============================================================

def build_tasks(windows: Dict[int, Dict[int, Tuple[float, float]]]) -> list:
    """
    Κατασκευάζει tasks με την ίδια δομή με το __main__ του thesis.py,
    αλλά με πραγματικά windows από το SUMO.

    AV 1 → task1 + task2
    AV 2 → task3
    """

    tasks = []

    # ------------------------------------------------------------
    # AV 1
    # ------------------------------------------------------------
    av1_windows = windows.get(1, {})

    if av1_windows:
        max_exit = max(t_exit for _, t_exit in av1_windows.values())
        first_enter = min(t_enter for t_enter, _ in av1_windows.values())

        task1 = Task(
            task_id=1,
            av_id=1,
            tau=max_exit,
            subtasks=[
                SubTask(
                    subtask_id=1,
                    parent_task_id=1,
                    D_in=1.5e6,
                    D_out=2.0e4,
                    omega=2.0e8,
                    deadline=max_exit,
                    order=0,
                    earliest_start=0.00,
                    bs_time_windows=av1_windows,
                ),
                SubTask(
                    subtask_id=2,
                    parent_task_id=1,
                    D_in=1.0e6,
                    D_out=2.0e4,
                    omega=1.5e8,
                    deadline=max_exit,
                    order=1,
                    earliest_start=0.00,
                    bs_time_windows=av1_windows,
                ),
            ],
        )

        task2 = Task(
            task_id=2,
            av_id=1,
            tau=max_exit,
            subtasks=[
                SubTask(
                    subtask_id=1,
                    parent_task_id=2,
                    D_in=1.0e6,
                    D_out=2.0e4,
                    omega=1.5e8,
                    deadline=max_exit,
                    order=0,
                    earliest_start=first_enter,
                    bs_time_windows=av1_windows,
                ),
                SubTask(
                    subtask_id=2,
                    parent_task_id=2,
                    D_in=0.8e6,
                    D_out=2.0e4,
                    omega=1.0e8,
                    deadline=max_exit,
                    order=1,
                    earliest_start=first_enter,
                    bs_time_windows=av1_windows,
                ),
            ],
        )

        tasks.extend([task1, task2])

    else:
        print("AV 1: δεν συλλέχθηκαν windows — παραλείπεται")

    # ------------------------------------------------------------
    # AV 2
    # ------------------------------------------------------------
    av2_windows = windows.get(2, {})

    if av2_windows:
        max_exit = max(t_exit for _, t_exit in av2_windows.values())

        task3 = Task(
            task_id=3,
            av_id=2,
            tau=max_exit,
            subtasks=[
                SubTask(
                    subtask_id=1,
                    parent_task_id=3,
                    D_in=1.2e6,
                    D_out=2.0e4,
                    omega=2.0e8,
                    deadline=max_exit,
                    order=0,
                    earliest_start=0.00,
                    bs_time_windows=av2_windows,
                ),
                SubTask(
                    subtask_id=2,
                    parent_task_id=3,
                    D_in=1.0e6,
                    D_out=2.0e4,
                    omega=1.5e8,
                    deadline=max_exit,
                    order=1,
                    earliest_start=0.00,
                    bs_time_windows=av2_windows,
                ),
            ],
        )

        tasks.append(task3)

    else:
        print("AV 2: δεν συλλέχθηκαν windows — παραλείπεται")

    return tasks


def print_tasks_summary(tasks: list) -> None:
    """
    Εκτυπώνει τα tasks/subtasks που δημιουργήθηκαν πριν τρέξει ο RASMHA.
    """

    print("\nTasks/SubTasks που δημιουργήθηκαν:")
    print("-" * 65)

    for task in tasks:
        print(f"\nTask {task.task_id} από AV{task.av_id}")
        print(f"  tau = {task.tau:.2f}s")

        print("  διαθέσιμα BS windows:")
        for bs_id, (t_enter, t_exit) in task.subtasks[0].bs_time_windows.items():
            print(f"    BS{bs_id}: [{t_enter:.2f}, {t_exit:.2f}] s")

        print("  Subtasks:")
        for st in sorted(task.subtasks, key=lambda s: s.order):
            print(
                f"    SubTask {st.subtask_id} | "
                f"order={st.order} | "
                f"D_in={st.D_in:.1e} bits | "
                f"D_out={st.D_out:.1e} bits | "
                f"omega={st.omega:.1e} cycles | "
                f"earliest_start={st.earliest_start:.2f}s | "
                f"deadline={st.deadline:.2f}s"
            )


def print_results(result, tasks: list) -> None:
    """
    Εκτυπώνει τα αποτελέσματα του RASMHA.
    """

    print("\n" + "=" * 65)
    print("  RASMHA — Route-Aware Subtask MEC Heuristic Algorithm")
    print("=" * 65)

    for task in tasks:
        i = task.task_id

        print(f"\nTask {i}  (deadline τ = {task.tau:.2f} s)")
        print(
            f"  {'SubTask':<10} {'BS':>4} {'t_start':>10} "
            f"{'t_finish':>10} {'delay':>10} {'V2I window'}"
        )
        print("  " + "-" * 60)

        for st in sorted(task.subtasks, key=lambda s: s.order):
            key = (i, st.subtask_id)

            if key in result.assignments:
                bs = result.assignments[key]
                ts = result.start_times[key]
                tf = result.finish_times[key]
                d = result.D[key]
                win = st.bs_time_windows.get(bs, ("?", "?"))
                win_str = f"[{win[0]:.2f}, {win[1]:.2f}]"

                print(
                    f"  Sub {st.subtask_id:<6} BS {bs:>2}  "
                    f"{ts:>9.2f}s  {tf:>9.2f}s  {d:>9.4f}s  {win_str}"
                )

            else:
                print(f"  Sub {st.subtask_id:<6} *** INFEASIBLE ***")

        delay = result.task_delays[i]
        ok = not any(tid == i for tid, _ in result.failed_subtasks)

        status = "feasible" if ok and delay <= task.tau else "deadline missed"

        print(f"  Total task delay: {delay:.4f} s   [{status}]")

    print()

    if result.failed_subtasks:
        print(f"Failed subtasks : {result.failed_subtasks}")
    else:
        print("All subtasks assigned successfully.")

    print(f"Total system cost (feasible tasks only): {result.total_cost:.4f} s")


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    print("── Phase 1: SUMO simulation ──────────────────────────")
    windows = run_sumo()

    print("\nWindows που συλλέχθηκαν:")
    for av_id, av_windows in windows.items():
        if not av_windows:
            print(f"  AV{av_id}: κανένα window")
            continue

        for bs_id, (t_enter, t_exit) in av_windows.items():
            print(f"  AV{av_id} ↔ BS{bs_id}: [{t_enter:.2f}, {t_exit:.2f}] s")

    print("\n── Phase 2: RASMHA scheduling ────────────────────────")
    tasks = build_tasks(windows)

    if not tasks:
        print("Δεν υπάρχουν tasks για scheduling.")
        sys.exit(0)

    print_tasks_summary(tasks)

    R_av_bs = {
        (av_id, bs_id): V2I_RATE
        for av_id, av_windows in windows.items()
        for bs_id in av_windows
    }

    print("\nV2I rates που θα χρησιμοποιηθούν:")
    for (av_id, bs_id), rate in R_av_bs.items():
        print(f"  AV{av_id} ↔ BS{bs_id}: {rate:.2e} bps")

    result = mobility_aware_task_offloading_main(
        tasks=tasks,
        servers=SERVERS,
        R_av_bs=R_av_bs,
        method="RASMHA",
    )

    print_results(result, tasks)