from __future__ import annotations

from dataclasses import dataclass
from math import inf
from typing import Dict, List, Tuple, Literal, Optional


# ============================================================
# Data structures
# ============================================================

@dataclass
class Task:
    """
    u_i = {D_in, D_out, omega, task_type, tau}
    """
    task_id: int
    D_in: float          # input data size (bits)
    D_out: float         # result data size (bits)
    omega: float         # required CPU cycles
    task_type: int       # task type
    tau: float           # maximum allowable delay
    F_u: float           # local CPU frequency of MU i
    P_tx: float          # transmission power of MU i


@dataclass
class MECServer:
    """
    MEC server on BS j
    """
    bs_id: int
    F_total: float       # total CPU frequency of MEC server
    C_max: int           # max simultaneous tasks

    @property
    def f_alloc(self) -> float:
        """
        Article assumption:
        MEC computational resources are evenly allocated to the maximum
        number of tasks that the MEC server can process simultaneously.
        """
        if self.C_max <= 0:
            raise ValueError(f"C_max must be > 0 for BS {self.bs_id}")
        return self.F_total / self.C_max


@dataclass
class SystemInputs:
    tasks: List[Task]
    servers: Dict[int, MECServer]

    # V[(i, j, k)] = 1:
    # for task i, access BS = j and migration BS = k
    V: Dict[Tuple[int, int, int], int]

    # Q[(i, j)] = 1: cache hit for task i at BS j
    Q: Dict[Tuple[int, int], int]

    # T_w[(i, j)] = residence time of MU i at access BS j
    T_w: Dict[Tuple[int, int], float]

    # R_mu_bs[(i, j)] = rate between MU i and BS j
    R_mu_bs: Dict[Tuple[int, int], float]

    # R_bs_bs[(j, k)] = rate between BS j and BS k
    R_bs_bs: Dict[Tuple[int, int], float]

    alpha: float
    epsilon: float = 1.25e-26


@dataclass
class AlgorithmOutput:
    Y: Dict[Tuple[int, int], int]
    Z: Dict[Tuple[int, int, int], int]
    D: Dict[int, float]
    total_cost: float


# ============================================================
# Helpers
# ============================================================

def get_access_and_migration_bs(
    V: Dict[Tuple[int, int, int], int],
    task_id: int
) -> Tuple[int, int]:
    """
    Returns (j, k) such that v_ijk = 1.
    """
    for (i, j, k), value in V.items():
        if i == task_id and value == 1:
            return j, k
    raise ValueError(f"No (access_bs, migration_bs) found in V for task {task_id}")


def get_task_by_id(tasks: List[Task], task_id: int) -> Task:
    for task in tasks:
        if task.task_id == task_id:
            return task
    raise ValueError(f"Task {task_id} not found")


# ============================================================
# Delay model
# ============================================================

def upload_delay(task: Task, R_ij: float) -> float:
    # Eq. (7)
    return task.D_in / R_ij


def local_compute_delay(task: Task) -> float:
    # Eq. (8)
    return task.omega / task.F_u


def mec_compute_delay(task: Task, server: MECServer) -> float:
    # Eq. (9)
    return task.omega / server.f_alloc


def inter_bs_delay(task: Task, R_jk: float) -> float:
    # Eq. (11)
    return task.D_in / R_jk


def return_delay(task: Task, R_ij: float) -> float:
    # Eq. (12)
    return task.D_out / R_ij


def access_cache_hit_delay(task: Task, R_ij: float) -> float:
    # Eq. (13)
    return return_delay(task, R_ij)


def access_cache_miss_delay(task: Task, server: MECServer, R_ij: float) -> float:
    # Eq. (14)
    return upload_delay(task, R_ij) + mec_compute_delay(task, server) + return_delay(task, R_ij)


def migration_cache_hit_delay(task: Task, R_ik: float, T_w_ij: float) -> float:
    # Eq. (15)
    return return_delay(task, R_ik) + T_w_ij


def migration_cache_miss_delay(
    task: Task,
    R_ij: float,
    R_jk: float,
    migration_server: MECServer,
    R_ik: float,
    T_w_ij: float,
) -> float:
    # Eq. (16)
    return max(
        upload_delay(task, R_ij)
        + inter_bs_delay(task, R_jk)
        + mec_compute_delay(task, migration_server)
        + return_delay(task, R_ik),
        T_w_ij + return_delay(task, R_ik),
    )


# ============================================================
# Energy model
# ============================================================

def local_energy(task: Task, epsilon: float) -> float:
    # Eq. (18)
    phi_i = epsilon * (task.F_u ** 2)
    return task.omega * phi_i


def offloading_energy(task: Task, R_ij: float) -> float:
    # Eq. (19)
    return task.P_tx * upload_delay(task, R_ij)


# ============================================================
# Cost functions
# ============================================================

def local_cost(task: Task, alpha: float, epsilon: float) -> float:
    Tl = local_compute_delay(task)
    El = local_energy(task, epsilon)
    return alpha * Tl + (1 - alpha) * El


def offload_cost_access_hit(task: Task, R_ij: float, alpha: float) -> float:
    Th = access_cache_hit_delay(task, R_ij)
    Et = offloading_energy(task, R_ij)
    return alpha * Th + (1 - alpha) * Et


def offload_cost_access_miss(task: Task, server: MECServer, R_ij: float, alpha: float) -> float:
    Tz = access_cache_miss_delay(task, server, R_ij)
    Et = offloading_energy(task, R_ij)
    return alpha * Tz + (1 - alpha) * Et


def offload_cost_migration_as_written_in_paper(
    task: Task,
    R_ij: float,
    R_ik: float,
    T_w_ij: float,
    alpha: float,
) -> float:
    """
    The paper's Eq. (23) and Algorithm 1 write the same expression for
    both q_ik = 1 and q_ik = 0. We preserve that structure faithfully.
    """
    Th_ik = migration_cache_hit_delay(task, R_ik, T_w_ij)
    Et_ij = offloading_energy(task, R_ij)
    return alpha * Th_ik + (1 - alpha) * Et_ij


# ============================================================
# Exact evaluation from final decisions Y, Z
# ============================================================

def evaluate_task_delay_energy(
    task: Task,
    system: SystemInputs,
    Y: Dict[Tuple[int, int], int],
    Z: Dict[Tuple[int, int, int], int],
) -> Tuple[float, float]:
    i = task.task_id
    j, k = get_access_and_migration_bs(system.V, i)

    y_ij = Y.get((i, j), 0)
    z_ijk = Z.get((i, j, k), 0)

    R_ij = system.R_mu_bs[(i, j)]
    R_ik = system.R_mu_bs[(i, k)]
    R_jk = system.R_bs_bs[(j, k)]
    T_w_ij = system.T_w[(i, j)]

    access_server = system.servers[j]
    migration_server = system.servers[k]

    q_ij = system.Q.get((i, j), 0)
    q_ik = system.Q.get((i, k), 0)

    # Local processing
    if y_ij == 0:
        T_i = local_compute_delay(task)
        E_i = local_energy(task, system.epsilon)
        return T_i, E_i

    # Offloading energy is always based on upload from MU to access BS
    E_i = offloading_energy(task, R_ij)

    # No migration
    if z_ijk == 0:
        if q_ij == 1:
            T_i = access_cache_hit_delay(task, R_ij)
        else:
            T_i = access_cache_miss_delay(task, access_server, R_ij)
        return T_i, E_i

    # Migration
    if q_ik == 1:
        T_i = migration_cache_hit_delay(task, R_ik, T_w_ij)
    else:
        T_i = migration_cache_miss_delay(
            task=task,
            R_ij=R_ij,
            R_jk=R_jk,
            migration_server=migration_server,
            R_ik=R_ik,
            T_w_ij=T_w_ij,
        )

    return T_i, E_i


def evaluate_total_cost(
    system: SystemInputs,
    Y: Dict[Tuple[int, int], int],
    Z: Dict[Tuple[int, int, int], int],
) -> AlgorithmOutput:
    D: Dict[int, float] = {}

    for task in system.tasks:
        T_i, E_i = evaluate_task_delay_energy(task, system, Y, Z)
        D[task.task_id] = system.alpha * T_i + (1 - system.alpha) * E_i

    return AlgorithmOutput(
        Y=Y,
        Z=Z,
        D=D,
        total_cost=sum(D.values()),
    )


# ============================================================
# CMHA - Algorithm 1
# ============================================================

def cmha(system: SystemInputs) -> AlgorithmOutput:
    """
    Cache-assist Mobility-aware Heuristic Algorithm (CMHA)

    Practical corrections applied:
    1) loop control uses decided_tasks < N
    2) capacity check uses the ACTUAL compute BS
    3) capacity check uses < instead of <=
    """
    alpha = system.alpha

    # C_N[j] = current number of tasks actually computed at BS j
    C_N: Dict[int, int] = {j: 0 for j in system.servers.keys()}

    # G[(i, bs, layer)] where layer in {1,2}
    # layer=1 -> local processing cost
    # layer=2 -> offloading/migration cost
    G: Dict[Tuple[int, int, int], float] = {}

    Y: Dict[Tuple[int, int], int] = {}
    Z: Dict[Tuple[int, int, int], int] = {}
    D: Dict[int, float] = {}

    # initialize all matrix entries to +inf
    for task in system.tasks:
        i = task.task_id
        for bs_id in system.servers.keys():
            G[(i, bs_id, 1)] = inf
            G[(i, bs_id, 2)] = inf

    # --------------------------------------------------------
    # Generate cost matrix
    # --------------------------------------------------------
    for task in system.tasks:
        i = task.task_id
        j, k = get_access_and_migration_bs(system.V, i)

        R_ij = system.R_mu_bs[(i, j)]
        R_ik = system.R_mu_bs[(i, k)]
        T_w_ij = system.T_w[(i, j)]

        access_server = system.servers[j]

        # layer 1: local processing cost
        G[(i, j, 1)] = local_cost(task, alpha, system.epsilon)

        # layer 2: offloading / migration cost
        if system.Q.get((i, j), 0) == 1:
            # access BS cache hit
            G[(i, j, 2)] = offload_cost_access_hit(task, R_ij, alpha)
        else:
            Tz_ij = access_cache_miss_delay(task, access_server, R_ij)

            if Tz_ij <= T_w_ij:
                # no migration, access BS computes
                G[(i, j, 2)] = offload_cost_access_miss(task, access_server, R_ij, alpha)
            else:
                # migration BS candidate
                G[(i, k, 2)] = offload_cost_migration_as_written_in_paper(
                    task=task,
                    R_ij=R_ij,
                    R_ik=R_ik,
                    T_w_ij=T_w_ij,
                    alpha=alpha,
                )

    # --------------------------------------------------------
    # Make decisions greedily
    # --------------------------------------------------------
    undecided = {task.task_id for task in system.tasks}
    decided_tasks = 0
    total_tasks = len(system.tasks)

    while decided_tasks < total_tasks:
        x, y, layer = min(G, key=G.get)
        g_xyz = G[(x, y, layer)]

        if g_xyz == inf:
            break

        if x not in undecided:
            G[(x, y, layer)] = inf
            continue

        task = get_task_by_id(system.tasks, x)
        j, k = get_access_and_migration_bs(system.V, x)

        # ----------------------------------------------------
        # local processing selected
        # ----------------------------------------------------
        if layer == 1:
            G[(x, j, 1)] = inf
            G[(x, j, 2)] = inf
            G[(x, k, 2)] = inf

            D[x] = g_xyz
            Y[(x, j)] = 0
            Z[(x, j, k)] = 0

            undecided.remove(x)
            decided_tasks += 1
            continue

        # ----------------------------------------------------
        # offloading/migration selected
        # ----------------------------------------------------
        q_xy = system.Q.get((x, y), 0)

        # cache hit branch
        if q_xy == 1:
            D[x] = g_xyz

            G[(x, j, 1)] = inf
            G[(x, j, 2)] = inf
            G[(x, k, 2)] = inf

            if y == j:
                Y[(x, j)] = 1
                Z[(x, j, k)] = 0
            else:
                Y[(x, j)] = 1
                Z[(x, j, k)] = 1

            undecided.remove(x)
            decided_tasks += 1
            continue

        # cache miss branch
        # IMPORTANT FIX:
        # the actual compute server is the selected BS y
        # - if y == j: no migration, access BS computes
        # - if y == k: migration, migration BS computes
        compute_bs = y

        # IMPORTANT FIX:
        # must be <, not <=
        if C_N[compute_bs] < system.servers[compute_bs].C_max:
            C_N[compute_bs] += 1
            D[x] = g_xyz

            G[(x, j, 1)] = inf
            G[(x, j, 2)] = inf
            G[(x, k, 2)] = inf

            if y == j:
                Y[(x, j)] = 1
                Z[(x, j, k)] = 0
            else:
                Y[(x, j)] = 1
                Z[(x, j, k)] = 1

            undecided.remove(x)
            decided_tasks += 1
        else:
            # reject only this offloading option
            G[(x, y, 2)] = inf

    # fallback to local if a task remains undecided
    for task in system.tasks:
        i = task.task_id
        if i in undecided:
            j, k = get_access_and_migration_bs(system.V, i)
            Y[(i, j)] = 0
            Z[(i, j, k)] = 0
            D[i] = local_cost(task, system.alpha, system.epsilon)

    return AlgorithmOutput(
        Y=Y,
        Z=Z,
        D=D,
        total_cost=sum(D.values()),
    )


# ============================================================
# ILP / OPTA skeleton
# ============================================================

def build_ilp_problem(system: SystemInputs):
    return {
        "objective": "min alpha * sum(T_i) + (1-alpha) * sum(E_i)",
        "decision_variables": ["Y_ij", "Z_ijk"],
        "constraints": [
            "C1: z_ijk <= (1 - q_ij)",
            "C2: T_i <= tau_i",
            "C3: z_ijk <= v_ijk",
            "C4: migration consistency",
            "C5: sum_j y_ij <= 1",
            "C6: C_N_j <= C_MAX_j",
        ],
    }


def solve_with_opta(ilp_problem):
    raise NotImplementedError(
        "OPTA solver not implemented here. Connect this with CPLEX / Gurobi / PuLP."
    )


# ============================================================
# Algorithm 2 support functions
# ============================================================

def build_trajectory_prediction_model(mu_history_data):
    """
    Placeholder for trajectory prediction model.
    """
    return {
        "model": "trajectory_prediction_placeholder",
        "history": mu_history_data,
    }


def information_gathering(
    current_mu_locations,
    prediction_model,
    tasks: List[Task],
    servers: Dict[int, MECServer],
    prebuilt_V: Dict[Tuple[int, int, int], int],
    prebuilt_Q: Dict[Tuple[int, int], int],
    prebuilt_Tw: Dict[Tuple[int, int], float],
):
    """
    Placeholder for the information gathering stage.
    """
    return {
        "tasks": tasks,
        "servers": servers,
        "V": prebuilt_V,
        "Q": prebuilt_Q,
        "T_w": prebuilt_Tw,
        "current_mu_locations": current_mu_locations,
        "prediction_model": prediction_model,
    }


def execute_decisions(
    Y: Dict[Tuple[int, int], int],
    Z: Dict[Tuple[int, int, int], int],
) -> None:
    """
    Placeholder for execution stage.
    """
    pass


def transmit_results_to_mu() -> None:
    """
    Placeholder for result transmission stage.
    """
    pass


# ============================================================
# Unified Algorithm 2 main
# ============================================================

def mobility_aware_task_offloading_main(
    tasks: List[Task],
    servers: Dict[int, MECServer],
    V: Dict[Tuple[int, int, int], int],
    Q: Dict[Tuple[int, int], int],
    T_w: Dict[Tuple[int, int], float],
    R_mu_bs: Dict[Tuple[int, int], float],
    R_bs_bs: Dict[Tuple[int, int], float],
    alpha: float,
    method: Literal["CMHA", "OPTA"] = "CMHA",
    mu_history_data=None,
    current_mu_locations=None,
) -> AlgorithmOutput:
    """
    Algorithm 2 main flow:
    - trajectory prediction
    - information gathering
    - choose OPTA or CMHA
    - execute decisions
    - transmit results
    """
    prediction_model = build_trajectory_prediction_model(mu_history_data)

    gathered = information_gathering(
        current_mu_locations=current_mu_locations,
        prediction_model=prediction_model,
        tasks=tasks,
        servers=servers,
        prebuilt_V=V,
        prebuilt_Q=Q,
        prebuilt_Tw=T_w,
    )

    system = SystemInputs(
        tasks=gathered["tasks"],
        servers=gathered["servers"],
        V=gathered["V"],
        Q=gathered["Q"],
        T_w=gathered["T_w"],
        R_mu_bs=R_mu_bs,
        R_bs_bs=R_bs_bs,
        alpha=alpha,
    )

    if method.upper() == "OPTA":
        ilp_problem = build_ilp_problem(system)
        Y, Z = solve_with_opta(ilp_problem)
        result = evaluate_total_cost(system, Y, Z)

    elif method.upper() == "CMHA":
        result = cmha(system)

    else:
        raise ValueError("method must be either 'CMHA' or 'OPTA'")

    execute_decisions(result.Y, result.Z)
    transmit_results_to_mu()

    return result


# ============================================================
# Example usage
# ============================================================

if __name__ == "__main__":
    tasks = [
        Task(
            task_id=1,
            D_in=1.5e6,
            D_out=2.0e4,
            omega=2.0e8,
            task_type=1,
            tau=1.0,
            F_u=0.7e9,
            P_tx=0.15,
        ),
        Task(
            task_id=2,
            D_in=1.2e6,
            D_out=2.0e4,
            omega=2.0e8,
            task_type=2,
            tau=1.2,
            F_u=0.6e9,
            P_tx=0.12,
        ),
    ]

    servers = {
        1: MECServer(bs_id=1, F_total=40e9, C_max=10),
        2: MECServer(bs_id=2, F_total=40e9, C_max=10),
        3: MECServer(bs_id=3, F_total=40e9, C_max=10),
    }

    # For each task i define one access BS and one migration BS
    V = {
        (1, 1, 2): 1,
        (2, 2, 3): 1,
    }

    # Cache matrix
    Q = {
        (1, 1): 0,
        (1, 2): 1,
        (2, 2): 0,
        (2, 3): 0,
    }

    # Residence time
    T_w = {
        (1, 1): 0.20,
        (2, 2): 0.30,
    }

    # MU-BS rates
    R_mu_bs = {
        (1, 1): 1.0e7,
        (1, 2): 1.0e7,
        (2, 2): 1.0e7,
        (2, 3): 1.0e7,
    }

    # BS-BS rates
    R_bs_bs = {
        (1, 2): 1.0e8,
        (2, 3): 1.0e8,
    }

    result = mobility_aware_task_offloading_main(
        tasks=tasks,
        servers=servers,
        V=V,
        Q=Q,
        T_w=T_w,
        R_mu_bs=R_mu_bs,
        R_bs_bs=R_bs_bs,
        alpha=0.5,
        method="CMHA",
        mu_history_data=None,
        current_mu_locations=None,
    )

    print("Y =", result.Y)
    print("Z =", result.Z)
    print("D =", result.D)
    print("total_cost =", result.total_cost)