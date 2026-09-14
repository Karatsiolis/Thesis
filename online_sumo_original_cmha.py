"""
online_sumo_original_cmha.py
=============================

SUMO wrapper for the ORIGINAL CMHA implementation.

IMPORTANT
---------
This file does not reimplement or modify CMHA. It dynamically imports
``algorithm_original.py`` and calls its original ``cmha(system)`` function.
All CMHA decisions (local execution, access-BS offloading, cache hit/miss,
migration, greedy cost matrix G, Y/Z decisions, delay-energy objective) are
therefore produced by the original algorithm code.

The wrapper only adds:
  * the same SUMO scenario and task generation used by RasmhaLast_recovery.py,
  * conversion of each ready RASMHA subtask to the Task input expected by CMHA,
  * construction of V, Q, T_w, communication rates and device parameters,
  * execution/evaluation of the unchanged CMHA decision in SUMO,
  * a terminal report with metrics comparable to RASMHA.

The original CMHA is coarse-grained. For a common experimental workload,
each ready RASMHA subtask is presented externally to CMHA as one independent
CMHA task. Sequential dependencies are enforced by this wrapper; the CMHA
function itself is unchanged.

RASMHA recovery is NOT copied into CMHA. The recovery implementation is used
only as the authoritative source of the common SUMO scenario, configuration,
vehicles, MEC servers and generated workload. CMHA keeps its original local,
access-BS and migration decisions so that the baseline is not modified.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from importlib.util import module_from_spec, spec_from_file_location
from math import inf
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
import hashlib
import random
import sys


# ============================================================
# Dynamic loading: original files remain untouched
# ============================================================

BASE_DIR = Path(__file__).resolve().parent


def _find_file(candidates: List[str]) -> Path:
    searched: List[Path] = []
    for filename in candidates:
        path = BASE_DIR / filename
        searched.append(path)
        if path.exists():
            return path
    raise FileNotFoundError(
        "Δεν βρέθηκε κανένα από τα απαιτούμενα αρχεία:\n"
        + "\n".join(str(path) for path in searched)
    )


def _load_module(module_name: str, path: Path):
    spec = spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Αδυναμία φόρτωσης module από {path}")
    module = module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


ALGORITHM_FILE = _find_file([
    "algorithm.py"
])

# The comparison must use the current recovery-enabled RASMHA implementation
# as the authoritative source of the shared scenario and workload.  Both names
# are accepted so the file works before or after removing the download suffix.
RASMHA_FILE = _find_file([
    "RasmhaLast_recovery.py"
])

CMHA = _load_module("original_cmha_algorithm", ALGORITHM_FILE)
RASMHA = _load_module("rasmha_recovery_reference_scenario", RASMHA_FILE)


def _validate_rasmha_reference(module) -> None:
    required_attributes = [
        "traci",
        "VEHICLES",
        "BS_COORDS",
        "BS_COVERAGE_M",
        "V2I_RATE",
        "SERVERS",
        "TASK_TEMPLATES",
        "FULL_CONFIG",
        "TASK_GENERATION_CONFIG",
        "SubTask",
        "Task",
        "dist",
        "route_cumulative_distances",
        "av_route_distance",
        "get_next_subtask",
        "create_task_from_template",
        "prepare_sumo",
    ]

    missing = [
        attribute
        for attribute in required_attributes
        if not hasattr(module, attribute)
    ]

    if missing:
        raise AttributeError(
            f"Το {RASMHA_FILE.name} δεν είναι συμβατό με τον CMHA wrapper. "
            f"Λείπουν: {', '.join(missing)}"
        )


_validate_rasmha_reference(RASMHA)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


ORIGINAL_ALGORITHM_SHA256 = _sha256(ALGORITHM_FILE)
RASMHA_REFERENCE_SHA256 = _sha256(RASMHA_FILE)


# ============================================================
# Shared scenario and optional comparison settings
# ============================================================

traci = RASMHA.traci

VEHICLES = RASMHA.VEHICLES
BS_COORDS = RASMHA.BS_COORDS
BS_COVERAGE_M = RASMHA.BS_COVERAGE_M
V2I_RATE = RASMHA.V2I_RATE
RASMHA_SERVERS = RASMHA.SERVERS
TASK_TEMPLATES = RASMHA.TASK_TEMPLATES

COMPARISON_CONFIG = RASMHA.FULL_CONFIG.get("cmha_comparison", {})

ALPHA = float(COMPARISON_CONFIG.get("alpha", 0.5))
BS_TO_BS_RATE = float(COMPARISON_CONFIG.get("bs_to_bs_rate", 1.0e8))
CACHE_HIT_RATE = float(COMPARISON_CONFIG.get("cache_hit_rate", 0.0))
NUM_TASK_TYPES = int(COMPARISON_CONFIG.get("num_task_types", 10))

LOCAL_CPU_MIN, LOCAL_CPU_MAX = COMPARISON_CONFIG.get(
    "local_cpu_range", [0.5e9, 0.8e9]
)
TX_POWER_MIN, TX_POWER_MAX = COMPARISON_CONFIG.get(
    "tx_power_range", [0.1, 0.2]
)

LOCAL_CPU_MIN = float(LOCAL_CPU_MIN)
LOCAL_CPU_MAX = float(LOCAL_CPU_MAX)
TX_POWER_MIN = float(TX_POWER_MIN)
TX_POWER_MAX = float(TX_POWER_MAX)

TASK_SEED = int(RASMHA.TASK_GENERATION_CONFIG.get("seed", 42))
MAX_SIM_TIME = float(getattr(RASMHA, "MAX_SIM_TIME", 140.0))
WAIT_LOG_PERIOD = float(getattr(RASMHA, "WAIT_LOG_PERIOD", 5.0))

if not 0.0 <= CACHE_HIT_RATE <= 1.0:
    raise ValueError("cache_hit_rate must be between 0 and 1")
if NUM_TASK_TYPES <= 0:
    raise ValueError("num_task_types must be positive")


# ============================================================
# Wrapper-only runtime structures
# ============================================================

SubTask = RASMHA.SubTask
Task = RASMHA.Task


@dataclass
class DecisionPlan:
    """A decision returned by the unchanged CMHA function."""

    cmha_task_id: int
    task_id: int
    subtask_id: int
    av_id: int

    access_bs: int
    migration_bs: int
    compute_bs: Optional[int]
    mode: str  # LOCAL, ACCESS, MIGRATION
    cache_hit: bool

    residence_time: float
    decision_time: float

    exact_delay: float
    service_time: float
    energy: float
    selected_cost: float
    evaluated_cost: float

    upload_time: float
    return_time: float


@dataclass
class RunningExecution:
    plan: DecisionPlan
    start_time: float
    finish_time: float
    upload_end_time: Optional[float]
    capacity_bs: Optional[int]


@dataclass
class OnlineState:
    active_tasks: Dict[int, Task] = field(default_factory=dict)
    completed_tasks: Set[int] = field(default_factory=set)
    failed_tasks: Set[int] = field(default_factory=set)
    completed_subtasks: Set[Tuple[int, int]] = field(default_factory=set)
    decided_subtasks: Set[Tuple[int, int]] = field(default_factory=set)

    next_order: Dict[int, int] = field(default_factory=dict)
    last_finish: Dict[int, float] = field(default_factory=dict)

    pending_plans: Dict[Tuple[int, int], DecisionPlan] = field(default_factory=dict)
    running_executions: List[RunningExecution] = field(default_factory=list)

    assignments: Dict[Tuple[int, int], Optional[int]] = field(default_factory=dict)
    access_assignments: Dict[Tuple[int, int], int] = field(default_factory=dict)
    migration_pairs: Dict[Tuple[int, int], Tuple[int, int]] = field(default_factory=dict)
    execution_modes: Dict[Tuple[int, int], str] = field(default_factory=dict)
    cache_hits: Dict[Tuple[int, int], bool] = field(default_factory=dict)

    delays: Dict[Tuple[int, int], float] = field(default_factory=dict)
    service_times: Dict[Tuple[int, int], float] = field(default_factory=dict)
    energies: Dict[Tuple[int, int], float] = field(default_factory=dict)
    selected_costs: Dict[Tuple[int, int], float] = field(default_factory=dict)
    evaluated_costs: Dict[Tuple[int, int], float] = field(default_factory=dict)

    start_times: Dict[Tuple[int, int], float] = field(default_factory=dict)
    finish_times: Dict[Tuple[int, int], float] = field(default_factory=dict)
    task_delays: Dict[int, float] = field(default_factory=dict)

    failure_reasons: Dict[int, str] = field(default_factory=dict)
    failure_times: Dict[int, float] = field(default_factory=dict)
    failed_subtasks: List[Tuple[int, int]] = field(default_factory=list)

    mec_load: Dict[int, int] = field(default_factory=dict)
    last_wait_log: Dict[Tuple[int, int], float] = field(default_factory=dict)


# ============================================================
# Deterministic device and cache inputs for original CMHA
# ============================================================


def _stable_rng(*values: int) -> random.Random:
    seed = TASK_SEED
    for value in values:
        seed = (seed * 1_000_003 + int(value) * 97_409) & 0xFFFFFFFFFFFF
    return random.Random(seed)


def local_cpu_for_av(av_id: int) -> float:
    return _stable_rng(11, av_id).uniform(LOCAL_CPU_MIN, LOCAL_CPU_MAX)


def tx_power_for_av(av_id: int) -> float:
    return _stable_rng(17, av_id).uniform(TX_POWER_MIN, TX_POWER_MAX)


def task_type_for_subtask(task_id: int, subtask_id: int) -> int:
    return ((task_id * 37 + subtask_id * 13) % NUM_TASK_TYPES) + 1


def build_cache_catalog() -> Dict[Tuple[int, int], bool]:
    """Deterministic cache state indexed by (BS, task_type)."""
    catalog: Dict[Tuple[int, int], bool] = {}
    for bs_id in sorted(RASMHA_SERVERS):
        for task_type in range(1, NUM_TASK_TYPES + 1):
            rng = _stable_rng(23, bs_id, task_type)
            catalog[(bs_id, task_type)] = rng.random() < CACHE_HIT_RATE
    return catalog


CACHE_CATALOG = build_cache_catalog()


# ============================================================
# Route geometry used only to construct V and T_w inputs
# ============================================================


def compute_route_coverage_intervals(
    route_polyline: List[Tuple[float, float]],
    bs_pos: Tuple[float, float],
    radius: float,
) -> List[Tuple[float, float]]:
    """Route-distance intervals lying inside a circular BS coverage area."""
    cumulative = RASMHA.route_cumulative_distances(route_polyline)
    raw: List[Tuple[float, float]] = []
    cx, cy = bs_pos

    for index in range(1, len(route_polyline)):
        ax, ay = route_polyline[index - 1]
        bx, by = route_polyline[index]
        dx = bx - ax
        dy = by - ay
        segment_length = (dx * dx + dy * dy) ** 0.5
        if segment_length <= 1e-12:
            continue

        fx = ax - cx
        fy = ay - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        discriminant = b * b - 4.0 * a * c

        cuts = [0.0, 1.0]
        if discriminant >= 0.0:
            root = max(0.0, discriminant) ** 0.5
            t1 = (-b - root) / (2.0 * a)
            t2 = (-b + root) / (2.0 * a)
            if 0.0 < t1 < 1.0:
                cuts.append(t1)
            if 0.0 < t2 < 1.0:
                cuts.append(t2)

        cuts = sorted(set(round(value, 12) for value in cuts))

        for left, right in zip(cuts[:-1], cuts[1:]):
            midpoint = (left + right) / 2.0
            mx = ax + midpoint * dx
            my = ay + midpoint * dy
            inside = (mx - cx) ** 2 + (my - cy) ** 2 <= radius ** 2 + 1e-9
            if inside:
                raw.append((
                    cumulative[index - 1] + left * segment_length,
                    cumulative[index - 1] + right * segment_length,
                ))

    if not raw:
        return []

    raw.sort()
    merged: List[List[float]] = []
    for start, end in raw:
        if not merged or start > merged[-1][1] + 1e-6:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)

    return [(start, end) for start, end in merged]


def current_interval(
    intervals: List[Tuple[float, float]],
    route_distance: float,
) -> Optional[Tuple[float, float]]:
    for start, end in intervals:
        if start - 1e-3 <= route_distance <= end + 1e-3:
            return start, end
    return None


def next_interval(
    intervals: List[Tuple[float, float]],
    route_distance: float,
) -> Optional[Tuple[float, float]]:
    for start, end in intervals:
        if end > route_distance + 1e-3:
            return start, end
    return None


def select_access_bs(
    av_pos: Tuple[float, float],
    reachable_bs: List[int],
    bs_xy: Dict[int, Tuple[float, float]],
) -> Optional[int]:
    if not reachable_bs:
        return None
    return min(reachable_bs, key=lambda bs: RASMHA.dist(av_pos, bs_xy[bs]))


def find_migration_bs(
    access_bs: int,
    current_route_distance: float,
    route_candidates: List[int],
    passed_bs: Set[int],
    intervals_by_bs: Dict[int, List[Tuple[float, float]]],
) -> Optional[int]:
    future: List[Tuple[float, int]] = []
    for bs_id in route_candidates:
        if bs_id == access_bs or bs_id in passed_bs:
            continue
        interval = next_interval(intervals_by_bs.get(bs_id, []), current_route_distance)
        if interval is None:
            continue
        entry, end = interval
        if end <= current_route_distance + 1e-3:
            continue
        future.append((max(entry, current_route_distance), bs_id))

    if not future:
        return None
    future.sort(key=lambda item: (item[0], item[1]))
    return future[0][1]


def residence_time_input(
    access_bs: int,
    current_route_distance: float,
    avg_speed: float,
    intervals_by_bs: Dict[int, List[Tuple[float, float]]],
) -> float:
    interval = current_interval(
        intervals_by_bs.get(access_bs, []),
        current_route_distance,
    )
    if interval is None:
        return 0.0
    _, exit_distance = interval
    return max(0.0, exit_distance - current_route_distance) / max(avg_speed, 1e-9)


# ============================================================
# Unchanged CMHA invocation
# ============================================================


def original_servers() -> Dict[int, object]:
    return {
        bs_id: CMHA.MECServer(
            bs_id=bs_id,
            F_total=server.F_total,
            C_max=server.C_max,
        )
        for bs_id, server in RASMHA_SERVERS.items()
    }


def build_and_run_original_cmha(
    candidates: List[Tuple[Task, SubTask, int, int, float]],
) -> List[DecisionPlan]:
    """
    Build the inputs expected by algorithm_original.py and call its cmha()
    exactly as implemented. No CMHA condition or cost formula is duplicated here.
    """
    if not candidates:
        return []

    cmha_tasks: List[object] = []
    V: Dict[Tuple[int, int, int], int] = {}
    Q: Dict[Tuple[int, int], int] = {}
    T_w: Dict[Tuple[int, int], float] = {}
    R_mu_bs: Dict[Tuple[int, int], float] = {}
    R_bs_bs: Dict[Tuple[int, int], float] = {}
    metadata: Dict[int, Tuple[Task, SubTask, int, int, float]] = {}

    servers = original_servers()

    for parent_task, subtask, access_bs, migration_bs, residence in candidates:
        cmha_task_id = parent_task.task_id * 1000 + subtask.subtask_id
        task_type = task_type_for_subtask(parent_task.task_id, subtask.subtask_id)

        cmha_task = CMHA.Task(
            task_id=cmha_task_id,
            D_in=subtask.D_in,
            D_out=subtask.D_out,
            omega=subtask.omega,
            task_type=task_type,
            tau=max(0.0, subtask.deadline - parent_task.generated_at),
            F_u=local_cpu_for_av(parent_task.av_id),
            P_tx=tx_power_for_av(parent_task.av_id),
        )
        cmha_tasks.append(cmha_task)
        metadata[cmha_task_id] = (
            parent_task,
            subtask,
            access_bs,
            migration_bs,
            residence,
        )

        V[(cmha_task_id, access_bs, migration_bs)] = 1
        Q[(cmha_task_id, access_bs)] = int(
            CACHE_CATALOG[(access_bs, task_type)]
        )
        Q[(cmha_task_id, migration_bs)] = int(
            CACHE_CATALOG[(migration_bs, task_type)]
        )
        T_w[(cmha_task_id, access_bs)] = residence
        R_mu_bs[(cmha_task_id, access_bs)] = V2I_RATE
        R_mu_bs[(cmha_task_id, migration_bs)] = V2I_RATE
        R_bs_bs[(access_bs, migration_bs)] = BS_TO_BS_RATE

    system = CMHA.SystemInputs(
        tasks=cmha_tasks,
        servers=servers,
        V=V,
        Q=Q,
        T_w=T_w,
        R_mu_bs=R_mu_bs,
        R_bs_bs=R_bs_bs,
        alpha=ALPHA,
    )

    # This is the original function from the uploaded algorithm file.
    result = CMHA.cmha(system)

    plans: List[DecisionPlan] = []
    for cmha_task in cmha_tasks:
        cmha_id = cmha_task.task_id
        parent_task, subtask, access_bs, migration_bs, residence = metadata[cmha_id]

        y = result.Y.get((cmha_id, access_bs), 0)
        z = result.Z.get((cmha_id, access_bs, migration_bs), 0)

        if y == 0:
            mode = "LOCAL"
            compute_bs: Optional[int] = None
            cache_hit = False
        elif z == 1:
            mode = "MIGRATION"
            compute_bs = migration_bs
            cache_hit = bool(Q.get((cmha_id, migration_bs), 0))
        else:
            mode = "ACCESS"
            compute_bs = access_bs
            cache_hit = bool(Q.get((cmha_id, access_bs), 0))

        exact_delay, energy = CMHA.evaluate_task_delay_energy(
            cmha_task,
            system,
            result.Y,
            result.Z,
        )

        upload = CMHA.upload_delay(cmha_task, V2I_RATE)
        returned = CMHA.return_delay(cmha_task, V2I_RATE)

        if mode == "LOCAL":
            service = CMHA.local_compute_delay(cmha_task)
        elif mode == "ACCESS" and cache_hit:
            service = returned
        elif mode == "ACCESS":
            service = (
                upload
                + CMHA.mec_compute_delay(cmha_task, servers[access_bs])
                + returned
            )
        elif cache_hit:
            service = returned
        else:
            service = (
                upload
                + CMHA.inter_bs_delay(cmha_task, BS_TO_BS_RATE)
                + CMHA.mec_compute_delay(cmha_task, servers[migration_bs])
                + returned
            )

        evaluated_cost = ALPHA * exact_delay + (1.0 - ALPHA) * energy

        plans.append(DecisionPlan(
            cmha_task_id=cmha_id,
            task_id=parent_task.task_id,
            subtask_id=subtask.subtask_id,
            av_id=parent_task.av_id,
            access_bs=access_bs,
            migration_bs=migration_bs,
            compute_bs=compute_bs,
            mode=mode,
            cache_hit=cache_hit,
            residence_time=residence,
            decision_time=0.0,  # assigned by caller
            exact_delay=exact_delay,
            service_time=service,
            energy=energy,
            selected_cost=result.D[cmha_id],
            evaluated_cost=evaluated_cost,
            upload_time=upload,
            return_time=returned,
        ))

    return plans


# ============================================================
# Execution of CMHA decisions in real SUMO time
# ============================================================


def get_subtask(task: Task, subtask_id: int) -> SubTask:
    for subtask in task.subtasks:
        if subtask.subtask_id == subtask_id:
            return subtask
    raise ValueError(f"SubTask {subtask_id} not found in Task {task.task_id}")


def fail_task(
    task: Task,
    state: OnlineState,
    current_time: float,
    reason: str,
) -> None:
    if task.task_id in state.failed_tasks or task.task_id in state.completed_tasks:
        return

    state.failed_tasks.add(task.task_id)
    state.failure_reasons[task.task_id] = reason
    state.failure_times[task.task_id] = current_time

    for key in list(state.pending_plans):
        if key[0] == task.task_id:
            state.pending_plans.pop(key, None)

    for execution in list(state.running_executions):
        if execution.plan.task_id != task.task_id:
            continue
        state.running_executions.remove(execution)
        if execution.capacity_bs is not None:
            state.mec_load[execution.capacity_bs] = max(
                0,
                state.mec_load.get(execution.capacity_bs, 0) - 1,
            )

    current_order = state.next_order.get(task.task_id, 0)
    for subtask in sorted(task.subtasks, key=lambda item: item.order):
        if subtask.order >= current_order:
            key = (task.task_id, subtask.subtask_id)
            if key not in state.failed_subtasks:
                state.failed_subtasks.append(key)

    print(f"  Task {task.task_id} από AV{task.av_id} ΑΠΕΤΥΧΕ: {reason}")


def complete_execution(
    execution: RunningExecution,
    state: OnlineState,
) -> None:
    plan = execution.plan
    task = state.active_tasks[plan.task_id]
    key = (plan.task_id, plan.subtask_id)

    if execution.capacity_bs is not None:
        state.mec_load[execution.capacity_bs] = max(
            0,
            state.mec_load.get(execution.capacity_bs, 0) - 1,
        )

    state.completed_subtasks.add(key)
    state.finish_times[key] = execution.finish_time
    state.last_finish[plan.task_id] = execution.finish_time
    state.next_order[plan.task_id] += 1

    print(
        f"  t={execution.finish_time:.2f}s  Task {plan.task_id} / "
        f"SubTask {plan.subtask_id} ολοκληρώθηκε με CMHA mode={plan.mode}"
    )

    next_subtask = RASMHA.get_next_subtask(
        task,
        state.next_order[plan.task_id],
    )
    if next_subtask is None:
        total_delay = execution.finish_time - task.generated_at
        state.task_delays[plan.task_id] = total_delay
        state.completed_tasks.add(plan.task_id)
        print(
            f"  t={execution.finish_time:.2f}s  Task {plan.task_id} "
            f"ΟΛΟΚΛΗΡΩΘΗΚΕ | total_delay={total_delay:.4f}s"
        )


def plan_requires_capacity(plan: DecisionPlan) -> bool:
    return (
        plan.compute_bs is not None
        and not plan.cache_hit
        and plan.mode in {"ACCESS", "MIGRATION"}
    )


def start_pending_plans(
    current_time: float,
    state: OnlineState,
    current_reachable: Dict[int, List[int]],
) -> None:
    for key, plan in list(state.pending_plans.items()):
        if plan.task_id in state.failed_tasks or plan.task_id in state.completed_tasks:
            state.pending_plans.pop(key, None)
            continue

        task = state.active_tasks[plan.task_id]
        subtask = get_subtask(task, plan.subtask_id)

        if current_time > subtask.deadline:
            fail_task(task, state, current_time, "χάθηκε το subtask deadline πριν την εκτέλεση")
            continue
        if current_time > task.generated_at + task.tau:
            fail_task(task, state, current_time, "χάθηκε το συνολικό tau πριν την εκτέλεση")
            continue

        if plan.mode in {"ACCESS", "MIGRATION"}:
            if plan.access_bs not in current_reachable.get(plan.av_id, []):
                fail_task(
                    task,
                    state,
                    current_time,
                    reason=(
                        f"το AV{plan.av_id} έφυγε από το Access BS{plan.access_bs} "
                        f"πριν ξεκινήσει η απόφαση του CMHA"
                    ),
                )
                continue

        capacity_bs: Optional[int] = None
        if plan_requires_capacity(plan):
            assert plan.compute_bs is not None
            server = RASMHA_SERVERS[plan.compute_bs]
            if state.mec_load.get(plan.compute_bs, 0) >= server.C_max:
                last_log = state.last_wait_log.get(key, -inf)
                if current_time - last_log >= WAIT_LOG_PERIOD:
                    state.last_wait_log[key] = current_time
                    print(
                        f"  t={current_time:.1f}s  Task {plan.task_id} / "
                        f"SubTask {plan.subtask_id} περιμένει capacity στο "
                        f"BS{plan.compute_bs}; η απόφαση CMHA παραμένει αμετάβλητη"
                    )
                continue
            capacity_bs = plan.compute_bs
            state.mec_load[capacity_bs] = state.mec_load.get(capacity_bs, 0) + 1

        finish_time = current_time + plan.exact_delay

        # Καταγράφεται μόνο ως διαγνωστική χρονοσφραγίδα (πότε ολοκληρώνεται το
        # T^t_ij της Εξ. 7). Δεν αποτελεί κριτήριο αποτυχίας: το άρθρο δεν
        # ορίζει τέτοιο κριτήριο - βλ. αναλυτικό σχόλιο στην
        # update_running_executions().
        upload_end: Optional[float] = None
        if plan.mode == "MIGRATION" and not plan.cache_hit:
            upload_end = current_time + plan.upload_time

        execution = RunningExecution(
            plan=plan,
            start_time=current_time,
            finish_time=finish_time,
            upload_end_time=upload_end,
            capacity_bs=capacity_bs,
        )
        state.running_executions.append(execution)
        state.pending_plans.pop(key, None)
        state.start_times[key] = current_time

        print(
            f"  t={current_time:.2f}s  Task {plan.task_id} / SubTask {plan.subtask_id} "
            f"-> ORIGINAL CMHA {plan.mode}"
        )
        print(
            f"            Access=BS{plan.access_bs} | Migration=BS{plan.migration_bs} | "
            f"cache_hit={plan.cache_hit} | T_w={plan.residence_time:.4f}s | "
            f"delay={plan.exact_delay:.4f}s | finish={finish_time:.2f}s"
        )


def update_running_executions(
    current_time: float,
    state: OnlineState,
    current_reachable: Dict[int, List[int]],
) -> None:
    for execution in list(state.running_executions):
        plan = execution.plan
        task = state.active_tasks[plan.task_id]
        subtask = get_subtask(task, plan.subtask_id)

        if plan.task_id in state.failed_tasks:
            state.running_executions.remove(execution)
            continue

        if current_time > subtask.deadline or current_time > task.generated_at + task.tau:
            fail_task(
                task,
                state,
                current_time,
                reason=f"η απόφαση {plan.mode} του CMHA έχασε deadline/tau",
            )
            continue

        # ΣΗΜΕΙΩΣΗ ΠΙΣΤΟΤΗΤΑΣ ΣΤΟ ΑΡΘΡΟ (Lai et al., Ad Hoc Networks 156/2024):
        #
        # Εδώ υπήρχε έλεγχος που ακύρωνε ένα MIGRATION αν το AV είχε βγει από
        # την κάλυψη του Access BS τη στιγμή ολοκλήρωσης του upload. Ο έλεγχος
        # αυτός ΔΕΝ προβλέπεται πουθενά στο μοντέλο του άρθρου και μάλιστα
        # αντιβαίνει στη λογική του:
        #
        #   - Ο περιορισμός C4 (§3.6.3) ορίζει ότι το migration ενεργοποιείται
        #     ΑΚΡΙΒΩΣ όταν T^z_ij >= T^w_ij, δηλαδή όταν ο χρόνος επεξεργασίας
        #     στο Access BS υπερβαίνει τον χρόνο παραμονής του MU εκεί. Το
        #     upload (T^t_ij) είναι συστατικό του T^z_ij. Άρα το άρθρο ΠΕΡΙΜΕΝΕΙ
        #     το όχημα να φεύγει από το Access BS κατά τη διάρκεια της
        #     διεκπεραίωσης· αυτός ακριβώς είναι ο λόγος ύπαρξης του migration.
        #
        #   - Η Εξ. (16) ήδη ενσωματώνει πλήρως αυτή την κινητικότητα:
        #     T^h_ik = max{ T^t_ij + T^t_jk + T^c_ik + T^r_ik , T^w_ij + T^r_ik }
        #     Ο δεύτερος όρος δηλώνει ρητά ότι το MU λαμβάνει το αποτέλεσμα
        #     μόνο αφού εισέλθει στην κάλυψη του Migration BS.
        #
        # Επομένως η μοναδική συνθήκη επιτυχίας που ορίζει το άρθρο για το
        # migration είναι η κάλυψη στο Migration BS τη στιγμή επιστροφής του
        # αποτελέσματος - κάτι που ελέγχεται παρακάτω. Ο έλεγχος αφαιρέθηκε.
        if current_time < execution.finish_time:
            continue

        if plan.mode == "ACCESS":
            if plan.access_bs not in current_reachable.get(plan.av_id, []):
                fail_task(
                    task,
                    state,
                    current_time,
                    reason=(
                        f"το αποτέλεσμα δεν επέστρεψε επειδή το AV{plan.av_id} "
                        f"βγήκε από το Access BS{plan.access_bs}"
                    ),
                )
                continue

        elif plan.mode == "MIGRATION":
            # ΣΗΜΕΙΩΣΗ ΠΙΣΤΟΤΗΤΑΣ ΣΤΟ ΑΡΘΡΟ (§3.5.2, περιπτώσεις 4 & 5):
            #
            # "since the MU can only receive the task computation result when
            #  entering the Migration BS service coverage, the total delay is
            #  the sum of the computation result return delay and the residence
            #  time of MU i in Access BS" -> Εξ. (15) και Εξ. (16).
            #
            # Το άρθρο δηλαδή ΑΝΑΜΕΝΕΙ την είσοδο του MU στην κάλυψη του
            # Migration BS· δεν ακυρώνει την εργασία. Ο δεύτερος όρος του max
            # στην Εξ. (16), T^w_ij + T^r_ik, είναι ακριβώς αυτή η αναμονή.
            #
            # Στο μοντέλο του άρθρου (Fig. 1) οι κυψέλες είναι εφαπτόμενες, άρα
            # "έξοδος από Access BS" == "είσοδος σε Migration BS" και ο όρος
            # T^w_ij αρκεί. Στο σενάριο SUMO της παρούσας εργασίας οι καλύψεις
            # ΔΕΝ είναι συνεχείς (υπάρχει κενό μεταξύ BS2 και BS3), οπότε η
            # πραγματική στιγμή εισόδου είναι η πιστή γενίκευση του T^w_ij.
            if plan.migration_bs not in current_reachable.get(plan.av_id, []):
                continue

            # Πραγματική στιγμή παράδοσης: γενίκευση της Εξ. (16) με τον
            # μετρημένο χρόνο εισόδου αντί του υποτιθέμενου T^w_ij.
            if current_time > execution.finish_time:
                execution.finish_time = current_time

        state.running_executions.remove(execution)
        complete_execution(execution, state)


# ============================================================
# Main SUMO run
# ============================================================


def run_online_sumo_original_cmha() -> OnlineState:
    print("-- Online SUMO wrapper + ORIGINAL CMHA ----------------------------------")
    print(f"Original algorithm file  : {ALGORITHM_FILE.name}")
    print(f"Original algorithm SHA256: {ORIGINAL_ALGORITHM_SHA256}")
    print(f"RASMHA reference file    : {RASMHA_FILE.name}")
    print(f"RASMHA reference SHA256  : {RASMHA_REFERENCE_SHA256}")

    (
        bs_xy,
        av_edges,
        route_candidates_by_av,
        route_polylines,
        travel_times,
    ) = RASMHA.prepare_sumo()

    average_speeds: Dict[int, float] = {}
    coverage_intervals: Dict[int, Dict[int, List[Tuple[float, float]]]] = {}

    for av_id, polyline in route_polylines.items():
        route_length = RASMHA.route_cumulative_distances(polyline)[-1]
        average_speeds[av_id] = route_length / max(travel_times[av_id], 1e-9)
        coverage_intervals[av_id] = {
            bs_id: compute_route_coverage_intervals(
                polyline,
                bs_xy[bs_id],
                BS_COVERAGE_M,
            )
            for bs_id in BS_COORDS
        }

    state = OnlineState(mec_load={bs_id: 0 for bs_id in RASMHA_SERVERS})
    generated_task_ids: Set[int] = set()
    target_vehicles = {f"av{av_id}" for av_id in av_edges}

    in_range: Dict[int, Dict[int, bool]] = {
        av_id: {bs_id: False for bs_id in BS_COORDS}
        for av_id in av_edges
    }
    passed_bs: Dict[int, Set[int]] = {av_id: set() for av_id in av_edges}

    try:
        while traci.simulation.getTime() <= MAX_SIM_TIME:
            all_generated = len(generated_task_ids) == len(TASK_TEMPLATES)
            if (
                not target_vehicles
                and all_generated
                and not state.pending_plans
                and not state.running_executions
            ):
                break

            traci.simulationStep()
            current_time = traci.simulation.getTime()

            current_reachable: Dict[int, List[int]] = {
                av_id: [] for av_id in av_edges
            }
            av_positions: Dict[int, Tuple[float, float]] = {}
            vehicle_ids = set(traci.vehicle.getIDList())

            for av_id in av_edges:
                vehicle_id = f"av{av_id}"
                if vehicle_id not in vehicle_ids:
                    continue

                position = traci.vehicle.getPosition(vehicle_id)
                av_positions[av_id] = position

                for bs_id, bs_position in bs_xy.items():
                    inside = RASMHA.dist(position, bs_position) <= BS_COVERAGE_M
                    if inside:
                        current_reachable[av_id].append(bs_id)

                    if inside and not in_range[av_id][bs_id]:
                        in_range[av_id][bs_id] = True
                        print(f"  t={current_time:.1f}s  AV{av_id} -> εντός BS{bs_id}")
                    elif not inside and in_range[av_id][bs_id]:
                        in_range[av_id][bs_id] = False
                        passed_bs[av_id].add(bs_id)
                        print(f"  t={current_time:.1f}s  AV{av_id} -> εκτός BS{bs_id}")

            update_running_executions(current_time, state, current_reachable)

            arrived = set(traci.simulation.getArrivedIDList())
            for vehicle_id in list(target_vehicles):
                if vehicle_id not in arrived:
                    continue
                target_vehicles.remove(vehicle_id)
                av_id = int(vehicle_id.replace("av", ""))
                print(f"  t={current_time:.1f}s  {vehicle_id} έφτασε στον προορισμό")

                for task in list(state.active_tasks.values()):
                    if (
                        task.av_id == av_id
                        and task.task_id not in state.completed_tasks
                        and task.task_id not in state.failed_tasks
                    ):
                        fail_task(
                            task,
                            state,
                            current_time,
                            reason=(
                                f"το AV{av_id} έφτασε στον προορισμό πριν "
                                f"ολοκληρωθεί το task"
                            ),
                        )

            # Exactly the same templates and creation times as RasmhaLast_recovery.py.
            for template in TASK_TEMPLATES:
                if template.task_id in generated_task_ids:
                    continue
                if current_time < template.generate_at:
                    continue

                av_id = template.av_id
                vehicle_id = f"av{av_id}"
                if vehicle_id in traci.vehicle.getIDList():
                    task = RASMHA.create_task_from_template(template, current_time)
                    state.active_tasks[task.task_id] = task
                    state.next_order[task.task_id] = 0
                    state.last_finish[task.task_id] = task.generated_at
                    generated_task_ids.add(task.task_id)

                    print(
                        f"\n  t={current_time:.1f}s  Δημιουργήθηκε Task "
                        f"{task.task_id} από AV{task.av_id}"
                    )
                else:
                    generated_task_ids.add(template.task_id)
                    print(
                        f"  t={current_time:.1f}s  Task {template.task_id} δεν "
                        f"δημιουργήθηκε γιατί το AV{template.av_id} δεν υπάρχει"
                    )

            # Start decisions already returned by CMHA but waiting for capacity.
            start_pending_plans(current_time, state, current_reachable)

            # Build one batch of all ready, undecided subtasks and invoke the
            # original cmha() once for that batch. The function is untouched.
            batch_inputs: List[Tuple[Task, SubTask, int, int, float]] = []

            for task_id, task in list(state.active_tasks.items()):
                if task_id in state.completed_tasks or task_id in state.failed_tasks:
                    continue
                if any(execution.plan.task_id == task_id for execution in state.running_executions):
                    continue
                if any(key[0] == task_id for key in state.pending_plans):
                    continue

                subtask = RASMHA.get_next_subtask(task, state.next_order[task_id])
                if subtask is None:
                    continue

                key = (task_id, subtask.subtask_id)
                if key in state.decided_subtasks:
                    continue
                if current_time < subtask.earliest_start:
                    continue
                if current_time < state.last_finish.get(task_id, task.generated_at):
                    continue

                if current_time > subtask.deadline:
                    fail_task(task, state, current_time, "χάθηκε το subtask deadline")
                    continue
                if current_time > task.generated_at + task.tau:
                    fail_task(task, state, current_time, "χάθηκε το συνολικό tau")
                    continue

                av_id = task.av_id
                reachable = current_reachable.get(av_id, [])
                if not reachable or av_id not in av_positions:
                    last_log = state.last_wait_log.get(key, -inf)
                    if current_time - last_log >= WAIT_LOG_PERIOD:
                        state.last_wait_log[key] = current_time
                        print(
                            f"  t={current_time:.1f}s  Task {task_id} / "
                            f"SubTask {subtask.subtask_id} περιμένει Access BS"
                        )
                    continue

                access_bs = select_access_bs(av_positions[av_id], reachable, bs_xy)
                if access_bs is None:
                    continue

                route_distance = RASMHA.av_route_distance(
                    av_positions[av_id],
                    route_polylines[av_id],
                )
                migration_bs = find_migration_bs(
                    access_bs,
                    route_distance,
                    route_candidates_by_av.get(av_id, []),
                    passed_bs.get(av_id, set()),
                    coverage_intervals[av_id],
                )

                residence = residence_time_input(
                    access_bs,
                    route_distance,
                    average_speeds[av_id],
                    coverage_intervals[av_id],
                )

                # The original model requires one migration BS. At the final BS,
                # no future migration exists. We use k=j and infinite T_w only as
                # an input encoding that disables migration; cmha() is unchanged.
                if migration_bs is None:
                    migration_bs = access_bs
                    residence = inf

                batch_inputs.append((
                    task,
                    subtask,
                    access_bs,
                    migration_bs,
                    residence,
                ))

            plans = build_and_run_original_cmha(batch_inputs)
            for plan in plans:
                key = (plan.task_id, plan.subtask_id)
                if key in state.decided_subtasks:
                    continue

                plan.decision_time = current_time
                state.decided_subtasks.add(key)
                state.pending_plans[key] = plan

                state.assignments[key] = plan.compute_bs
                state.access_assignments[key] = plan.access_bs
                state.execution_modes[key] = plan.mode
                state.cache_hits[key] = plan.cache_hit
                if plan.mode == "MIGRATION":
                    state.migration_pairs[key] = (
                        plan.access_bs,
                        plan.migration_bs,
                    )

                state.delays[key] = plan.exact_delay
                state.service_times[key] = plan.service_time
                state.energies[key] = plan.energy
                state.selected_costs[key] = plan.selected_cost
                state.evaluated_costs[key] = plan.evaluated_cost

            start_pending_plans(current_time, state, current_reachable)

        for task in list(state.active_tasks.values()):
            if (
                task.task_id not in state.completed_tasks
                and task.task_id not in state.failed_tasks
            ):
                fail_task(
                    task,
                    state,
                    traci.simulation.getTime(),
                    reason="τέλος προσομοίωσης χωρίς ολοκλήρωση",
                )

    finally:
        traci.close()

    return state


# ============================================================
# Comparable terminal report
# ============================================================


def _median(values: List[float]) -> float:
    ordered = sorted(values)
    count = len(ordered)
    if count == 0:
        return 0.0
    middle = count // 2
    if count % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def print_final_report(state: OnlineState) -> None:
    width = 132
    print("\n" + "=" * width)
    print("ORIGINAL CMHA + SUMO - FINAL SIMULATION REPORT".center(width))
    print("=" * width)

    if not state.active_tasks:
        print("Δεν δημιουργήθηκαν tasks.")
        return

    tasks = [state.active_tasks[task_id] for task_id in sorted(state.active_tasks)]
    generated_tasks = len(tasks)
    generated_subtasks = sum(len(task.subtasks) for task in tasks)
    completed = sorted(state.completed_tasks)
    failed = sorted(state.failed_tasks)
    delays = [state.task_delays[task_id] for task_id in completed]

    print("\nSIMULATION CONFIGURATION")
    print("-" * width)
    print(f"{'Algorithm source':<34}: {ALGORITHM_FILE.name}")
    print(f"{'Algorithm SHA256':<34}: {ORIGINAL_ALGORITHM_SHA256}")
    print(f"{'Vehicles':<34}: {len(VEHICLES)}")
    print(f"{'MEC Servers':<34}: {len(RASMHA_SERVERS)}")
    print(f"{'Generated Tasks':<34}: {generated_tasks}")
    print(f"{'Generated SubTasks':<34}: {generated_subtasks}")
    print(f"{'V2I Rate':<34}: {V2I_RATE / 1e6:.2f} Mbps")
    print(f"{'BS-to-BS Rate':<34}: {BS_TO_BS_RATE / 1e6:.2f} Mbps")
    print(f"{'BS Coverage Radius':<34}: {BS_COVERAGE_M:.2f} m")
    print(f"{'CMHA alpha':<34}: {ALPHA:.2f}")
    print(f"{'Cache hit rate input':<34}: {CACHE_HIT_RATE * 100:.2f} %")

    print("\nTASK RESULTS")
    print("-" * width)
    header = (
        f"{'Task':>4}  {'AV':>3}  {'Execution Sequence':<25}  "
        f"{'Generated':>9}  {'End/Fail':>9}  {'Waiting':>9}  "
        f"{'Service':>9}  {'E2E Delay':>10}  {'Tau':>8}  {'Status':<18}"
    )
    print(header)
    print("-" * width)

    for task in tasks:
        ordered_subtasks = sorted(task.subtasks, key=lambda item: item.order)
        sequence: List[str] = []
        service = 0.0

        for subtask in ordered_subtasks:
            key = (task.task_id, subtask.subtask_id)
            mode = state.execution_modes.get(key)
            if mode == "LOCAL":
                sequence.append("LOCAL")
            elif mode == "MIGRATION":
                pair = state.migration_pairs.get(key)
                sequence.append(f"{pair[0]}>{pair[1]}" if pair else "MIG")
            elif mode == "ACCESS":
                compute_bs = state.assignments.get(key)
                sequence.append(str(compute_bs) if compute_bs is not None else "-")
            else:
                sequence.append("-")
            service += state.service_times.get(key, 0.0)

        sequence_text = " -> ".join(sequence)
        if task.task_id in state.completed_tasks:
            e2e = state.task_delays[task.task_id]
            end = max(
                state.finish_times[(task.task_id, subtask.subtask_id)]
                for subtask in ordered_subtasks
                if (task.task_id, subtask.subtask_id) in state.finish_times
            )
            waiting = max(0.0, e2e - service)
            status = "COMPLETED" if e2e <= task.tau else "TAU MISSED"
            end_text = f"{end:.2f}"
            waiting_text = f"{waiting:.4f}"
            e2e_text = f"{e2e:.4f}"
        elif task.task_id in state.failed_tasks:
            end_text = f"{state.failure_times.get(task.task_id, 0.0):.2f}"
            waiting_text = "-"
            e2e_text = "-"
            status = "FAILED"
        else:
            end_text = waiting_text = e2e_text = "-"
            status = "UNFINISHED"

        service_text = f"{service:.4f}" if service > 0 else "-"
        print(
            f"{task.task_id:>4}  {task.av_id:>3}  {sequence_text:<25.25}  "
            f"{task.generated_at:>9.2f}  {end_text:>9}  {waiting_text:>9}  "
            f"{service_text:>9}  {e2e_text:>10}  {task.tau:>8.2f}  {status:<18}"
        )

    print("\nSYSTEM SUMMARY")
    print("-" * width)
    completed_count = len(completed)
    failed_count = len(failed)
    success_rate = 100.0 * completed_count / generated_tasks if generated_tasks else 0.0
    failure_rate = 100.0 * failed_count / generated_tasks if generated_tasks else 0.0
    print(f"{'Completed Tasks':<34}: {completed_count}")
    print(f"{'Failed Tasks':<34}: {failed_count}")
    print(f"{'Success Rate':<34}: {success_rate:.2f} %")
    print(f"{'Failure Rate':<34}: {failure_rate:.2f} %")

    if delays:
        print(f"{'Mean E2E Delay':<34}: {sum(delays) / len(delays):.4f} s")
        print(f"{'Median E2E Delay':<34}: {_median(delays):.4f} s")
        print(f"{'Minimum E2E Delay':<34}: {min(delays):.4f} s")
        print(f"{'Maximum E2E Delay':<34}: {max(delays):.4f} s")
        print(f"{'Total Completed E2E Delay':<34}: {sum(delays):.4f} s")
    else:
        for label in (
            "Mean E2E Delay",
            "Median E2E Delay",
            "Minimum E2E Delay",
            "Maximum E2E Delay",
            "Total Completed E2E Delay",
        ):
            print(f"{label:<34}: -")

    print("\nORIGINAL CMHA METRICS")
    print("-" * width)
    modes = list(state.execution_modes.values())
    cache_hits = sum(1 for hit in state.cache_hits.values() if hit)
    print(f"{'Local executions':<34}: {modes.count('LOCAL')}")
    print(f"{'Access-BS offloads':<34}: {modes.count('ACCESS')}")
    print(f"{'Migrations':<34}: {modes.count('MIGRATION')}")
    print(f"{'Cache hits':<34}: {cache_hits}")
    print(f"{'Total task energy':<34}: {sum(state.energies.values()):.6f}")
    print(f"{'CMHA selected total cost':<34}: {sum(state.selected_costs.values()):.6f}")
    print(f"{'Exact evaluated total cost':<34}: {sum(state.evaluated_costs.values()):.6f}")

    print("\nMEC UTILIZATION")
    print("-" * width)
    counts = {bs_id: 0 for bs_id in sorted(RASMHA_SERVERS)}
    for key, compute_bs in state.assignments.items():
        if compute_bs is not None:
            counts[compute_bs] += 1
    total_assignments = sum(counts.values())
    print(f"{'MEC':<8}{'Assigned SubTasks':>20}{'Share of Assignments':>24}")
    print("-" * 52)
    for bs_id in sorted(counts):
        share = 100.0 * counts[bs_id] / total_assignments if total_assignments else 0.0
        print(f"{('BS' + str(bs_id)):<8}{counts[bs_id]:>20}{share:>23.2f}%")
    print("-" * 52)
    print(f"{'Total':<8}{total_assignments:>20}{100.0 if total_assignments else 0.0:>23.2f}%")

    print("\nFAILURE ANALYSIS")
    print("-" * width)
    if not failed:
        print("No task failures were recorded.")
    else:
        for task_id in failed:
            task = state.active_tasks[task_id]
            current_subtask = RASMHA.get_next_subtask(
                task,
                state.next_order.get(task_id, 0),
            )
            subtask_text = current_subtask.subtask_id if current_subtask else "-"
            print(
                f"Task {task_id} / SubTask {subtask_text} | AV{task.av_id} | "
                f"t={state.failure_times.get(task_id, 0.0):.2f}s | "
                f"{state.failure_reasons.get(task_id, 'Unknown failure')}"
            )

    print("=" * width)


if __name__ == "__main__":
    final_state = run_online_sumo_original_cmha()
    print_final_report(final_state)
