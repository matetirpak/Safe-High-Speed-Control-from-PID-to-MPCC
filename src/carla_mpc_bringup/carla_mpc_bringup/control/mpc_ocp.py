"""Build the acados SQP-RTI solver for the trajectory-tracking OCP.

Take BicycleParams plus MpcConfig directly and return a ready AcadosOcpSolver.
The NONLINEAR_LS cost uses a per-stage parametric reference (the reference state
for that horizon step), with heading error wrapped to avoid the +-pi seam, box
constraints on the inputs, and an SQP-RTI solver with partial-condensing HPIPM.

The model runs in CARLA coordinates (see mpc_controller.py / docs/mpc.md).
acados writes c_generated_code/ in CWD, so generation happens inside ``gen_dir``.
"""
from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path

import numpy as np

from carla_mpc_bringup.control.vehicle_models import (
    BicycleParams,
    blended_bicycle,
    dynamic_bicycle_linear,
    kinematic_bicycle_cg,
    kinematic_bicycle_rear,
)
from carla_mpc_bringup.core.config_loader import MpcConfig

_BUILDERS = {
    "kin_cg": (kinematic_bicycle_cg, ["X", "Y", "psi", "v"]),
    "kin_rear": (kinematic_bicycle_rear, ["X", "Y", "psi", "v"]),
    "dyn": (dynamic_bicycle_linear, ["X", "Y", "psi", "vx", "vy", "r"]),
    "blended": (blended_bicycle, ["X", "Y", "psi", "vx", "vy", "r"]),
}
_DYNAMIC = ("dyn", "blended")


@contextmanager
def _chdir(path: Path):
    """Enter ``path`` (creating it) as CWD, restoring the previous CWD on exit."""
    prev = os.getcwd()
    path.mkdir(parents=True, exist_ok=True)
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)


def default_gen_dir(model_name: str) -> Path:
    """Return the per-model acados codegen cache dir (override via CARLA_MPC_ACADOS_DIR)."""
    base = os.environ.get("CARLA_MPC_ACADOS_DIR")
    root = Path(base) if base else (Path.home() / ".cache" / "carla_mpc_acados")
    return root / model_name


def build_tracking_solver(params: BicycleParams, cfg: MpcConfig, delta_max: float,
                          gen_dir: Path | None = None):
    """Generate, compile, and return the acados tracking solver.

    Parameters
    ----------
    params : BicycleParams
        Vehicle geometry/tyre parameters for the chosen bicycle model.
    cfg : MpcConfig
        Horizon, timestep, cost weights, and constraint bounds.
    delta_max : float
        Steering-angle box-constraint limit, rad (applied symmetrically).
    gen_dir : Path, optional
        Codegen directory; defaults to ``default_gen_dir(model.name)``.

    Returns
    -------
    solver : AcadosOcpSolver
        Ready SQP-RTI solver for the trajectory-tracking OCP.
    nx : int
        State dimension (4 for kinematic, 6 for dynamic models).
    N : int
        Prediction-horizon length, in steps.
    dt : float
        Horizon timestep, s.

    Raises
    ------
    ValueError
        If ``cfg.model`` is not a known bicycle-model key.
    """
    import casadi as ca
    import scipy.linalg
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

    if cfg.model not in _BUILDERS:
        raise ValueError(f"unknown mpc.model '{cfg.model}' (expected {list(_BUILDERS)})")
    builder, x_names = _BUILDERS[cfg.model]
    f_expl, x, u = builder(params)
    nx, nu = x.shape[0], u.shape[0]

    model = AcadosModel()
    model.name = f"carla_mpc_{cfg.model}"
    model.x, model.u = x, u
    model.xdot = ca.SX.sym("xdot", nx)
    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl

    is_dyn = cfg.model in _DYNAMIC
    ix, iy, ipsi = x_names.index("X"), x_names.index("Y"), x_names.index("psi")
    iv = x_names.index("vx") if is_dyn else x_names.index("v")

    # Parameter vector is reference [Xr, Yr, psir, vr] (4) plus last applied steering
    # delta_prev (1); independent of nx so it works for the 4- and 6-state models.
    # The (delta - delta_prev) residual penalises steering change to damp chatter.
    x_ref = ca.SX.sym("x_ref", 4)
    delta_prev = ca.SX.sym("delta_prev", 1)
    model.p = ca.vertcat(x_ref, delta_prev)

    # Contouring decomposition of the position error: rotate (X-Xref, Y-Yref) into
    # the path frame at psi_ref to get along-track (e_lon) and cross-track (e_lat).
    # Weight e_lat (stay on path) high and e_lon (pacing) low so the MPC picks its
    # own speed.
    dx, dy = x[ix] - x_ref[0], x[iy] - x_ref[1]
    cpsi, spsi = ca.cos(x_ref[2]), ca.sin(x_ref[2])
    e_lon = cpsi * dx + spsi * dy
    e_lat = -spsi * dx + cpsi * dy
    psi_err = ca.atan2(ca.sin(x[ipsi] - x_ref[2]), ca.cos(x[ipsi] - x_ref[2]))
    v_err = x[iv] - x_ref[3]
    ddelta = u[1] - delta_prev

    model.cost_y_expr = ca.vertcat(e_lon, e_lat, psi_err, v_err, u, ddelta)
    model.cost_y_expr_e = ca.vertcat(e_lon, e_lat, psi_err, v_err)

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = cfg.N
    ocp.solver_options.tf = cfg.N * cfg.dt
    ocp.parameter_values = np.zeros(5)         # [Xr, Yr, psir, vr, delta_prev]

    # Residual order: [e_lon, e_lat, psi_err, v_err, a, delta, ddelta].
    Qs = np.diag([cfg.q_lon, cfg.q_lat, cfg.q_psi, cfg.q_v])   # the 4 state residuals
    R = np.diag([cfg.r_a, cfg.r_delta])
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = scipy.linalg.block_diag(Qs, R, np.array([[cfg.r_ddelta]]))
    ocp.cost.W_e = cfg.q_terminal_scale * Qs
    ocp.cost.yref = np.zeros(4 + nu + 1)
    ocp.cost.yref_e = np.zeros(4)

    ocp.constraints.lbu = np.array([cfg.a_min, -delta_max])
    ocp.constraints.ubu = np.array([cfg.a_max, +delta_max])
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.x0 = np.zeros(nx)

    # Dynamic model: bound vx, vy, r so the unsaturated linear-tyre QP stays
    # well-conditioned; without these it blows up to NaN under aggressive correction.
    if is_dyn:
        ivx, ivy, ir = x_names.index("vx"), x_names.index("vy"), x_names.index("r")
        ocp.constraints.idxbx = np.array([ivx, ivy, ir])
        ocp.constraints.lbx = np.array([cfg.vx_min, -cfg.vy_max, -cfg.r_max])
        ocp.constraints.ubx = np.array([cfg.v_max, cfg.vy_max, cfg.r_max])

    # Soft constraint |a_lat| <= a_lat_max caps transient curve lateral acceleration.
    # kinematic: a_lat = v^2 sin(beta)/l_r; dynamic: a_lat = vx*r.
    if cfg.a_lat_max > 0:
        if is_dyn:
            a_lat = x[x_names.index("vx")] * x[x_names.index("r")]
        else:
            beta = ca.atan2(params.l_r * ca.tan(u[1]), params.L)
            a_lat = x[x_names.index("v")] ** 2 * ca.sin(beta) / params.l_r
        model.con_h_expr = a_lat
        ocp.constraints.lh = np.array([-cfg.a_lat_max])
        ocp.constraints.uh = np.array([+cfg.a_lat_max])
        ocp.constraints.idxsh = np.array([0])
        ocp.cost.zl = ocp.cost.zu = np.array([cfg.a_lat_slack])
        ocp.cost.Zl = ocp.cost.Zu = np.array([cfg.a_lat_slack])

    o = ocp.solver_options
    o.nlp_solver_type = "SQP_RTI"
    o.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    o.qp_solver_cond_N = max(1, cfg.N // 4)
    o.qp_solver_iter_max = 50
    o.qp_solver_warm_start = 1
    # Dynamic (stiff linear tyres) needs an implicit integrator; explicit ERK NaNs.
    o.integrator_type = "IRK" if is_dyn else "ERK"
    o.sim_method_num_stages = 3 if is_dyn else 4
    o.sim_method_num_steps = 3
    o.hessian_approx = "GAUSS_NEWTON"
    o.nlp_solver_max_iter = 1
    o.print_level = 0

    gen = gen_dir or default_gen_dir(model.name)
    with _chdir(gen):
        solver = AcadosOcpSolver(ocp, json_file=str(gen / f"{model.name}.json"),
                                 verbose=False)
    return solver, nx, cfg.N, cfg.dt
