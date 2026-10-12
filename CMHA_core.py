"""
Implementation of the CMHA algorithm (Cache-assist Mobility-aware Heuristic
Algorithm) from: Lai et al., "Mobility-aware task offloading in MEC with
task migration and result caching", Ad Hoc Networks 156, 2024.
Used as the baseline for the comparison with RASMHA.

Contents:
  - the delay model (Eq. 7-16) and the energy model (Eq. 18-19),
  - the cost functions: alpha * delay + (1 - alpha) * energy,
  - Algorithm 1 (CMHA): greedy choice between local execution,
    offloading to the Access BS and migration to the Migration BS,
  - the flow of Algorithm 2, with placeholders for the parts that are not
    implemented (OPTA/ILP solver, trajectory prediction).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import inf
from typing import Dict, List, Tuple, Literal, Optional


#---------------------------------------------------------
#Data structures
#---------------------------------------------------------

@dataclass
class Task:
    """Task u_i of mobile user (MU) i."""
    task_id: int
    D_in: float          # input data size (bits)
    D_out: float         # result size (bits)
    omega: float         # required CPU cycles
    task_type: int       # task type
    tau: float           # maximum allowed delay
    F_u: float           # CPU frequency of the MU (local execution)
    P_tx: float          # transmission power of the MU


@dataclass
class MECServer:
    """MEC server at base station (BS) j."""
    bs_id: int
    F_total: float       # total CPU frequency of the server
    C_max: int           # maximum number of concurrent tasks

    @property
    def f_alloc(self) -> float:
        # Paper assumption: MEC resources are split equally among the
        # C_max tasks the server can serve at the same time.
        if self.C_max <= 0:
            raise ValueError(f"C_max must be > 0 for BS {self.bs_id}")
        return self.F_total / self.C_max


@dataclass
class SystemInputs:
    """All problem inputs for one run of the algorithm."""
    tasks: List[Task]
    servers: Dict[int, MECServer]

    # V[(i, j, k)] = 1: for task i, Access BS = j and Migration BS = k
    V: Dict[Tuple[int, int, int], int]

    # Q[(i, j)] = 1: cache hit for task i at BS j
    Q: Dict[Tuple[int, int], int]

    # T_w[(i, j)]: residence time of MU i in the coverage of Access BS j
    T_w: Dict[Tuple[int, int], float]

    # R_mu_bs[(i, j)]: transmission rate between MU i and BS j
    R_mu_bs: Dict[Tuple[int, int], float]

    # R_bs_bs[(j, k)]: transmission rate between BS j and BS k
    R_bs_bs: Dict[Tuple[int, int], float]

    alpha: float                 # weight of delay versus energy
    epsilon: float = 1.25e-26    # energy coefficient of the processor


@dataclass
class AlgorithmOutput:
    """Decisions and cost returned by the algorithm."""
    Y: Dict[Tuple[int, int], int]           # y_ij = 1: offloading via Access BS j (0: local)
    Z: Dict[Tuple[int, int, int], int]      # z_ijk = 1: migration from j to k
    D: Dict[int, float]                     # cost per task
    total_cost: float


#---------------------------------------------------------
#Helper functions
#---------------------------------------------------------

def get_access_and_migration_bs(
    V: Dict[Tuple[int, int, int], int],
    task_id: int
) -> Tuple[int, int]:
    """Returns (j, k): the Access BS and the Migration BS of the task (v_ijk = 1)."""
    for (i, j, k), value in V.items():
        if i == task_id and value == 1:
            return j, k
    raise ValueError(f"No (access_bs, migration_bs) found in V for task {task_id}")


def get_task_by_id(tasks: List[Task], task_id: int) -> Task:
    for task in tasks:
        if task.task_id == task_id:
            return task
    raise ValueError(f"Task {task_id} not found")


#---------------------------------------------------------
#Delay model
#---------------------------------------------------------

def upload_delay(task: Task, R_ij: float) -> float:
    # Eq. (7): sending the input data from the MU to the BS
    return task.D_in / R_ij


def local_compute_delay(task: Task) -> float:
    # Eq. (8): local execution on the MU
    return task.omega / task.F_u


def mec_compute_delay(task: Task, server: MECServer) -> float:
    # Eq. (9): execution on the MEC server
    return task.omega / server.f_alloc


def inter_bs_delay(task: Task, R_jk: float) -> float:
    # Eq. (11): transferring the input data from BS j to BS k
    return task.D_in / R_jk


def return_delay(task: Task, R_ij: float) -> float:
    # Eq. (12): returning the result from the BS to the MU
    return task.D_out / R_ij


def access_cache_hit_delay(task: Task, R_ij: float) -> float:
    # Eq. (13): cache hit at the Access BS -> only the result return
    return return_delay(task, R_ij)


def access_cache_miss_delay(task: Task, server: MECServer, R_ij: float) -> float:
    # Eq. (14): cache miss at the Access BS -> upload + computation + return
    return upload_delay(task, R_ij) + mec_compute_delay(task, server) + return_delay(task, R_ij)


def migration_cache_hit_delay(task: Task, R_ik: float, T_w_ij: float) -> float:
    # Eq. (15): cache hit at the Migration BS -> the MU receives the result
    # after leaving the Access BS (T_w_ij) and entering the Migration BS
    return return_delay(task, R_ik) + T_w_ij


def migration_cache_miss_delay(
    task: Task,
    R_ij: float,
    R_jk: float,
    migration_server: MECServer,
    R_ik: float,
    T_w_ij: float,
) -> float:
    # Eq. (16): cache miss at the Migration BS -> the maximum of the service
    # time (upload + transfer j->k + computation + return) and the time
    # the MU needs to reach the Migration BS
    return max(
        upload_delay(task, R_ij)
        + inter_bs_delay(task, R_jk)
        + mec_compute_delay(task, migration_server)
        + return_delay(task, R_ik),
        T_w_ij + return_delay(task, R_ik),
    )


#---------------------------------------------------------
#Energy model
#---------------------------------------------------------
def local_energy(task: Task, epsilon: float) -> float:
    # Eq. (18): local execution energy = cycles * epsilon * F_u^2
    phi_i = epsilon * (task.F_u ** 2)
    return task.omega * phi_i


def offloading_energy(task: Task, R_ij: float) -> float:
    # Eq. (19): offloading energy = transmission power * upload time
    return task.P_tx * upload_delay(task, R_ij)


#---------------------------------------------------------
#Cost functions: alpha * delay + (1 - alpha) * energy
#---------------------------------------------------------
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
    Migration cost as written in Eq. (23) and Algorithm 1 of the paper:
    the same expression (delay of Eq. 15) is used for both q_ik = 1 and
    q_ik = 0. It is kept as is for fidelity to the paper.
    """
    Th_ik = migration_cache_hit_delay(task, R_ik, T_w_ij)
    Et_ij = offloading_energy(task, R_ij)
    return alpha * Th_ik + (1 - alpha) * Et_ij


#---------------------------------------------------------
#Exact evaluation of the final decisions Y, Z
#---------------------------------------------------------
def evaluate_task_delay_energy(
    task: Task,
    system: SystemInputs,
    Y: Dict[Tuple[int, int], int],
    Z: Dict[Tuple[int, int, int], int],
) -> Tuple[float, float]:
    """
    Computes the actual delay T_i and energy E_i of a task from the
    decisions Y, Z. Here migration with a cache miss is evaluated with the
    full Eq. (16).
    """
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

    # Local execution
    if y_ij == 0:
        T_i = local_compute_delay(task)
        E_i = local_energy(task, system.epsilon)
        return T_i, E_i

    # Offloading: the energy is always that of the upload to the Access BS
    E_i = offloading_energy(task, R_ij)

    # No migration: served by the Access BS
    if z_ijk == 0:
        if q_ij == 1:
            T_i = access_cache_hit_delay(task, R_ij)
        else:
            T_i = access_cache_miss_delay(task, access_server, R_ij)
        return T_i, E_i

    # With migration: served by the Migration BS
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
    """Cost of each task and total cost for the given decisions Y, Z."""
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


#---------------------------------------------------------
#CMHA - Algorithm 1
#---------------------------------------------------------
def cmha(system: SystemInputs) -> AlgorithmOutput:
    """
    Algorithm 1 of the paper (CMHA).

      1. Build the cost matrix G: for each task, one local execution cost
         (layer 1) and one offloading or migration cost (layer 2).
      2. Greedy decisions: each time the cheapest available entry of G is
         chosen, with a capacity check when MEC computation is needed.

    Corrections with respect to the paper's pseudocode:
      1) the loop continues while decided_tasks < N,
      2) capacity is checked at the BS that actually executes the task,
      3) the capacity check uses < instead of <=.
    """
    alpha = system.alpha

    # C_N[j]: number of tasks assigned for computation to BS j
    C_N: Dict[int, int] = {j: 0 for j in system.servers.keys()}

    # G[(i, bs, layer)]: layer 1 = local execution, layer 2 = offloading/migration
    G: Dict[Tuple[int, int, int], float] = {}

    Y: Dict[Tuple[int, int], int] = {}
    Z: Dict[Tuple[int, int, int], int] = {}
    D: Dict[int, float] = {}

    # Initially all entries of G are +inf (unavailable options)
    for task in system.tasks:
        i = task.task_id
        for bs_id in system.servers.keys():
            G[(i, bs_id, 1)] = inf
            G[(i, bs_id, 2)] = inf

    #--------------------------------------------------------
    #Step 1: build the cost matrix G
    #--------------------------------------------------------
    for task in system.tasks:
        i = task.task_id
        j, k = get_access_and_migration_bs(system.V, i)

        R_ij = system.R_mu_bs[(i, j)]
        R_ik = system.R_mu_bs[(i, k)]
        T_w_ij = system.T_w[(i, j)]

        access_server = system.servers[j]

        # Layer 1: local execution cost
        G[(i, j, 1)] = local_cost(task, alpha, system.epsilon)

        # Layer 2: offloading or migration cost
        if system.Q.get((i, j), 0) == 1:
            # Cache hit at the Access BS: no computation needed
            G[(i, j, 2)] = offload_cost_access_hit(task, R_ij, alpha)
        else:
            Tz_ij = access_cache_miss_delay(task, access_server, R_ij)

            if Tz_ij <= T_w_ij:
                # Service finishes before the MU leaves: execution at the Access BS
                G[(i, j, 2)] = offload_cost_access_miss(task, access_server, R_ij, alpha)
            else:
                # The MU will leave earlier: migration candidate at BS k
                G[(i, k, 2)] = offload_cost_migration_as_written_in_paper(
                    task=task,
                    R_ij=R_ij,
                    R_ik=R_ik,
                    T_w_ij=T_w_ij,
                    alpha=alpha,
                )

    #--------------------------------------------------------
    #Step 2: greedy decisions
    #--------------------------------------------------------
    undecided = {task.task_id for task in system.tasks}
    decided_tasks = 0
    total_tasks = len(system.tasks)

    while decided_tasks < total_tasks:
        # Cheapest available entry of G
        x, y, layer = min(G, key=G.get)
        g_xyz = G[(x, y, layer)]

        if g_xyz == inf:
            break

        # The task has already been decided: the entry is discarded
        if x not in undecided:
            G[(x, y, layer)] = inf
            continue

        task = get_task_by_id(system.tasks, x)
        j, k = get_access_and_migration_bs(system.V, x)

        #----------------------------------------------------
        #Local execution selected
        #----------------------------------------------------
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

        #----------------------------------------------------
        #Offloading or migration selected
        #----------------------------------------------------
        q_xy = system.Q.get((x, y), 0)

        # Cache hit: no MEC capacity is reserved
        if q_xy == 1:
            D[x] = g_xyz

            G[(x, j, 1)] = inf
            G[(x, j, 2)] = inf
            G[(x, k, 2)] = inf

            # y == j: served by the Access BS, otherwise migration to k
            if y == j:
                Y[(x, j)] = 1
                Z[(x, j, k)] = 0
            else:
                Y[(x, j)] = 1
                Z[(x, j, k)] = 1

            undecided.remove(x)
            decided_tasks += 1
            continue

        # Cache miss: the selected BS y executes the task
        # (y == j: Access BS without migration, y == k: Migration BS)
        compute_bs = y

        if C_N[compute_bs] < system.servers[compute_bs].C_max:
            # Free slot on the MEC: the decision is finalized
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
            # The MEC is full: only this option is rejected,
            # local execution remains available
            G[(x, y, 2)] = inf

    # Tasks left without a decision are executed locally
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


#---------------------------------------------------------
#ILP / OPTA (skeleton only, not solved)
#---------------------------------------------------------

def build_ilp_problem(system: SystemInputs):
    # Description of the paper's ILP: objective, decision variables, constraints C1-C6
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
    # The optimal OPTA solver is not implemented (requires an external solver)
    raise NotImplementedError(
        "OPTA solver not implemented here. Connect this with CPLEX / Gurobi / PuLP."
    )


#---------------------------------------------------------
#Stages of Algorithm 2 (placeholders)
#The inputs V, Q, T_w are provided ready-made by the caller,
#so these stages simply pass them through.
#---------------------------------------------------------
def build_trajectory_prediction_model(mu_history_data):
    # Placeholder for the trajectory prediction model
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
    # Placeholder for information gathering: returns the ready-made inputs
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
    """Placeholder for executing the decisions."""
    pass


def transmit_results_to_mu() -> None:
    """Placeholder for sending the results to the MU."""
    pass


#---------------------------------------------------------
#Algorithm 2 - main flow
#---------------------------------------------------------
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
    Algorithm 2 of the paper: trajectory prediction -> information gathering
    -> decision with OPTA or CMHA -> execute decisions -> send results.
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

    # Choice of decision method
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


#---------------------------------------------------------
#Example run: 2 tasks, 3 BSs
#---------------------------------------------------------

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

    # For each task i: (i, Access BS, Migration BS)
    V = {
        (1, 1, 2): 1,
        (2, 2, 3): 1,
    }

    # Cache: 1 = cache hit of the task at the BS
    Q = {
        (1, 1): 0,
        (1, 2): 1,
        (2, 2): 0,
        (2, 3): 0,
    }

    # Residence time at the Access BS (s)
    T_w = {
        (1, 1): 0.20,
        (2, 2): 0.30,
    }

    # MU-BS transmission rates (bits/s)
    R_mu_bs = {
        (1, 1): 1.0e7,
        (1, 2): 1.0e7,
        (2, 2): 1.0e7,
        (2, 3): 1.0e7,
    }

    # BS-BS transmission rates (bits/s)
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