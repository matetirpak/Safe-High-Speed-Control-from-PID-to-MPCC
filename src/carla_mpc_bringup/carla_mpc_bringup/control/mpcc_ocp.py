"""Build the acados SQP-RTI solver for the MPCC (Model Predictive Contouring Control) OCP.

State ``x = [X, Y, psi, vx, vy, r, theta]`` is a dynamic bicycle plus path progress
theta; input ``u = [a, delta, v_theta]`` adds the virtual progress speed, with
``theta_dot = v_theta``.

The reference path enters as per-stage parameters (not a baked-in interpolant), so the
solver never regenerates as the route is extended. Each stage gets a local
arc-linearisation of the path around a linearisation point ``theta_lin`` (the warm-start
theta_k): ``x_p(theta) = x_ref + cos(psi_ref) * (theta - theta_lin)``, etc. The contouring
error e_c and lag error e_l are taken against this local path; minimising e_c keeps the car
on the path, e_l keeps theta synced to the real arc length, and v_theta is driven toward a
per-stage target speed (set as yref by the controller, the friction-limited curve speed).

A friction-circle radius ``sqrt(a_lat^2 + a_long^2) <= a_grip`` is a soft safety backstop.
This linear-tyre plus friction-limited-feed-forward formulation is the robust controller; a
saturating-Pacejka tyre is too fragile for real-time SQP-RTI and must not be re-added here.

Per-stage parameter vector p = [x_ref, y_ref, psi_ref, kappa_ref, theta_lin, delta_prev].
"""
from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path

import numpy as np

from carla_mpc_bringup.control.vehicle_models import BicycleParams, blended_bicycle
from carla_mpc_bringup.core.config_loader import MpccConfig


@contextmanager
def _chdir(path: Path):
    prev = os.getcwd()
    path.mkdir(parents=True, exist_ok=True)
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)


def default_gen_dir(model_name: str) -> Path:
    base = os.environ.get("CARLA_MPC_ACADOS_DIR")
    root = Path(base) if base else (Path.home() / ".cache" / "carla_mpc_acados")
    return root / model_name


def build_mpcc_solver(params: BicycleParams, cfg: MpccConfig, delta_max: float,
                      gen_dir: Path | None = None):
    """Build the MPCC OCP and code-generate its acados solver.

    Parameters
    ----------
    params : BicycleParams
        Vehicle parameters for the blended kinematic/dynamic bicycle model.
    cfg : MpccConfig
        Horizon, timestep, cost weights, and box/grip limits.
    delta_max : float
        Steering-angle bound in rad, applied symmetrically.
    gen_dir : Path, optional
        Directory for acados code generation; defaults to the user cache.

    Returns
    -------
    tuple
        (AcadosOcpSolver, nx, N, dt) with nx=7, nu=3. The yref is left zeroed here
        and set per-stage by the controller (v_theta and vx target the
        friction-limited curve speed).
    """
    import casadi as ca
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

    # Blend kinematic (below ~v_blend, where slip is negligible) into dynamic bicycle.
    # The pure linear-tyre model is ill-conditioned at low vx because the slip angles
    # atan2(vy +- l*r, vx) are hypersensitive as vx -> 0, which causes a start-up steering
    # wobble; blending to kinematic at low speed gives the correct low-speed yaw.
    f6, x6, u6 = blended_bicycle(params, v_min=getattr(cfg, "v_blend", 4.0))  # 6-state f, x6=[X,Y,psi,vx,vy,r], u6=[a,delta]
    theta = ca.SX.sym("theta")
    v_theta = ca.SX.sym("v_theta")
    x = ca.vertcat(x6, theta)
    u = ca.vertcat(u6, v_theta)
    f = ca.vertcat(f6, v_theta)
    nx = 7

    model = AcadosModel()
    model.name = "carla_mpcc"
    model.x, model.u = x, u
    model.xdot = ca.SX.sym("xdot", nx)
    model.f_expl_expr = f
    model.f_impl_expr = model.xdot - f

    # Per-stage path params: x_ref, y_ref, psi_ref, kappa_ref, theta_lin, delta_prev.
    p = ca.SX.sym("p", 6)
    model.p = p
    xr, yr, psir, kr, th_lin, dprev = (p[0], p[1], p[2], p[3], p[4], p[5])

    D = theta - th_lin
    xp = xr + ca.cos(psir) * D                  # local arc-linearised path point
    yp = yr + ca.sin(psir) * D
    psip = psir + kr * D
    X, Y, psi, vx, vy, r = (x6[0], x6[1], x6[2], x6[3], x6[4], x6[5])
    a, delta = u6[0], u6[1]
    e_c = -(X - xp) * ca.sin(psip) + (Y - yp) * ca.cos(psip)   # contouring error, perpendicular to path
    e_l = (X - xp) * ca.cos(psip) + (Y - yp) * ca.sin(psip)    # lag error, along path

    # Residual order [e_c, e_l, a, delta, ddelta, v_theta, vx] must match the W diagonal
    # and the per-stage yref the controller sets (v_theta and vx toward the curve speed).
    model.cost_y_expr = ca.vertcat(e_c, e_l, a, delta, delta - dprev, v_theta, vx)
    model.cost_y_expr_e = ca.vertcat(e_c, e_l)

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = cfg.N
    ocp.solver_options.tf = cfg.N * cfg.dt
    ocp.parameter_values = np.zeros(6)

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag([cfg.q_c, cfg.q_l, cfg.r_a, cfg.r_delta, cfg.r_ddelta,
                          cfg.q_vtheta, cfg.q_vx])
    ocp.cost.W_e = np.diag([cfg.q_c, cfg.q_l])
    ocp.cost.yref = np.zeros(7)
    ocp.cost.yref_e = np.zeros(2)

    ocp.constraints.lbu = np.array([cfg.a_min, -delta_max, 0.0])
    ocp.constraints.ubu = np.array([cfg.a_max, +delta_max, cfg.v_theta_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    # State bounds keep the linear-tyre QP conditioned and vx away from the singular vx -> 0.
    ocp.constraints.idxbx = np.array([3, 4, 5])     # vx, vy, r
    ocp.constraints.lbx = np.array([cfg.vx_min, -cfg.vy_max, -cfg.r_max])
    ocp.constraints.ubx = np.array([cfg.v_max, cfg.vy_max, cfg.r_max])
    ocp.constraints.x0 = np.zeros(nx)

    # Friction-circle soft backstop: sqrt(a_lat^2 + a_long^2) <= a_grip; the 1e-2 keeps
    # the sqrt differentiable at the origin.
    model.con_h_expr = ca.vertcat(ca.sqrt((vx * r) ** 2 + a ** 2 + 1e-2))
    ocp.constraints.lh = np.array([-1e9])
    ocp.constraints.uh = np.array([cfg.a_grip])
    ocp.constraints.idxsh = np.array([0])
    ocp.cost.zl = ocp.cost.zu = np.array([cfg.grip_slack])
    ocp.cost.Zl = ocp.cost.Zu = np.array([cfg.grip_slack / 10.0])

    o = ocp.solver_options
    o.nlp_solver_type = "SQP_RTI"
    o.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    o.qp_solver_cond_N = max(1, cfg.N // 5)
    o.qp_solver_iter_max = 50
    o.qp_solver_warm_start = 1
    o.integrator_type = "IRK"           # implicit: the linear tyres make the dynamics stiff
    o.sim_method_num_stages = 3
    o.sim_method_num_steps = 3
    o.hessian_approx = "GAUSS_NEWTON"
    o.nlp_solver_max_iter = 1
    o.print_level = 0

    gen = gen_dir or default_gen_dir(model.name)
    with _chdir(gen):
        solver = AcadosOcpSolver(ocp, json_file=str(gen / f"{model.name}.json"),
                                 verbose=False)
    return solver, nx, cfg.N, cfg.dt
