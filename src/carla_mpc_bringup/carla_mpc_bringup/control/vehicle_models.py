"""CasADi-symbolic vehicle models for the acados MPC.

Each builder returns ``(f_expl, x, u)``: a symbolic continuous-time dynamics
expression plus the state and input symbols, ready to plug into acados as
``model.x``, ``model.u``, ``model.f_expl_expr``.

Exported builders:
    kinematic_bicycle_cg : 4-state CG-referenced kinematic bicycle.
    kinematic_bicycle_rear : 4-state rear-axle-referenced (no slip angle).
    dynamic_bicycle_linear : 6-state dynamic bicycle, linear tires.
    blended_bicycle : 6-state kinematic/dynamic sigmoid blend for low speed.

Conventions (consistent across all models; do not mix with the opposite):
    ROS REP-103 axes: body x forward, y left, z up.
    Slip angle alpha = (velocity direction) - (wheel heading), F_y = -C * alpha.
    delta is the front-wheel angle in radians, after the actuation map.
    Longitudinal accel a is body-frame, with v_x_dot = a + v_y * r.
    Use atan2 throughout, never atan, to stay well-defined off the +x axis.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Tuple

import casadi as ca
import numpy as np


# -----------------------------------------------------------------------------
# Parameter dataclasses
# -----------------------------------------------------------------------------


@dataclass
class BicycleParams:
    """Geometric and mass parameters shared by kinematic and dynamic models."""
    L: float          # wheelbase, m
    l_r: float        # rear axle to CG, m
    l_f: float = None # front axle to CG, m; if None, computed as L - l_r
    m: float = 1845.0     # mass, kg (dynamic only)
    Iz: float = 3745.0    # yaw inertia, kg*m^2 (dynamic only); ~ m*l_f*l_r for the Model 3
    Cf: float = 120000.0  # front cornering stiffness, N/rad (dynamic only)
    Cr: float = 150000.0  # rear cornering stiffness, N/rad (dynamic only)

    def __post_init__(self):
        if self.l_f is None:
            self.l_f = self.L - self.l_r
        assert abs(self.l_f + self.l_r - self.L) < 1e-6, "l_f + l_r != L"


# -----------------------------------------------------------------------------
# Kinematic bicycle, CG-referenced (default for trajectory tracking)
# -----------------------------------------------------------------------------


def kinematic_bicycle_cg(p: BicycleParams) -> Tuple[ca.SX, ca.SX, ca.SX]:
    """Build a 4-state kinematic bicycle referenced at the CG.

    State x = [X, Y, psi, v]; input u = [a, delta]. The body slip angle
    beta = atan2(l_r * tan(delta), L) shifts the velocity heading; this is the
    default model for trajectory tracking.

    Parameters
    ----------
    p : BicycleParams
        Uses L and l_r only.

    Returns
    -------
    tuple of casadi.SX
        (f_expl, x, u): continuous dynamics expression and state/input symbols.
    """
    X = ca.SX.sym("X")
    Y = ca.SX.sym("Y")
    psi = ca.SX.sym("psi")
    v = ca.SX.sym("v")
    x = ca.vertcat(X, Y, psi, v)

    a = ca.SX.sym("a")
    delta = ca.SX.sym("delta")
    u = ca.vertcat(a, delta)

    beta = ca.atan2(p.l_r * ca.tan(delta), p.L)
    f_expl = ca.vertcat(
        v * ca.cos(psi + beta),
        v * ca.sin(psi + beta),
        (v / p.l_r) * ca.sin(beta),
        a,
    )
    return f_expl, x, u


def kinematic_bicycle_rear(p: BicycleParams) -> Tuple[ca.SX, ca.SX, ca.SX]:
    """Build a 4-state kinematic bicycle referenced at the rear axle.

    Algebraically equivalent to kinematic_bicycle_cg up to a fixed CG offset,
    but drops the slip-angle term (as in openpilot's lateral MPC). State
    x = [X_r, Y_r, psi, v]; input u = [a, delta].

    Returns
    -------
    tuple of casadi.SX
        (f_expl, x, u): continuous dynamics expression and state/input symbols.
    """
    X = ca.SX.sym("X")
    Y = ca.SX.sym("Y")
    psi = ca.SX.sym("psi")
    v = ca.SX.sym("v")
    x = ca.vertcat(X, Y, psi, v)

    a = ca.SX.sym("a")
    delta = ca.SX.sym("delta")
    u = ca.vertcat(a, delta)

    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        (v / p.L) * ca.tan(delta),
        a,
    )
    return f_expl, x, u


# -----------------------------------------------------------------------------
# Dynamic bicycle, linear tires (default MPC model; also the MPCC base)
# -----------------------------------------------------------------------------


def dynamic_bicycle_linear(p: BicycleParams) -> Tuple[ca.SX, ca.SX, ca.SX]:
    """Build a 6-state dynamic bicycle with linear tires.

    State x = [X, Y, psi, v_x, v_y, r] (body-frame v_x, v_y; r = psi_dot);
    input u = [a, delta]. Slip angle alpha = (velocity direction) -
    (wheel heading), with F_y = -C * alpha.

    Singular at v_x = 0; for low-speed operation blend with the kinematic
    model via blended_bicycle.

    Returns
    -------
    tuple of casadi.SX
        (f_expl, x, u): continuous dynamics expression and state/input symbols.
    """
    X = ca.SX.sym("X")
    Y = ca.SX.sym("Y")
    psi = ca.SX.sym("psi")
    vx = ca.SX.sym("vx")
    vy = ca.SX.sym("vy")
    r = ca.SX.sym("r")
    x = ca.vertcat(X, Y, psi, vx, vy, r)

    a = ca.SX.sym("a")
    delta = ca.SX.sym("delta")
    u = ca.vertcat(a, delta)

    # atan2 (not atan) keeps slip angles well-defined except at the origin.
    alpha_f = ca.atan2(vy + p.l_f * r, vx) - delta
    alpha_r = ca.atan2(vy - p.l_r * r, vx)

    # Tire forces in the tire frame; F_yf is perpendicular to the front wheel.
    Fyf = -p.Cf * alpha_f
    Fyr = -p.Cr * alpha_r

    f_expl = ca.vertcat(
        vx * ca.cos(psi) - vy * ca.sin(psi),
        vx * ca.sin(psi) + vy * ca.cos(psi),
        r,
        a + vy * r,                                       # v_x_dot, with centripetal term
        -vx * r + (Fyf * ca.cos(delta) + Fyr) / p.m,      # v_y_dot, cos(delta) retained
        (p.l_f * Fyf * ca.cos(delta) - p.l_r * Fyr) / p.Iz,
    )
    return f_expl, x, u


def blended_bicycle(p: BicycleParams, v_min: float = 2.0, k: float = 5.0
                    ) -> Tuple[ca.SX, ca.SX, ca.SX]:
    """Build a 6-state model that smoothly blends kinematic and dynamic tires.

    Below ~v_min the dynamic model's atan2 is ill-conditioned, so the kinematic
    equivalent is blended in via a sigmoid in v_x. Smoother than a hard switch
    and suitable for MPCC. State and input are the dynamic model's 6-state form;
    the kinematic prediction is projected into the dynamic state.

    Parameters
    ----------
    v_min : float
        Sigmoid midpoint speed, m/s; below it the kinematic branch dominates.
    k : float
        Sigmoid steepness; larger is closer to a hard switch.

    Returns
    -------
    tuple of casadi.SX
        (f_expl, x, u): continuous dynamics expression and state/input symbols.
    """
    f_dyn, x, u = dynamic_bicycle_linear(p)
    X, Y, psi, vx, vy, r = x[0], x[1], x[2], x[3], x[4], x[5]
    a, delta = u[0], u[1]

    # Kinematic predictions in the body frame.
    beta = ca.atan2(p.l_r * ca.tan(delta), p.L)
    v_kin = ca.sqrt(vx * vx + vy * vy + 1e-6)
    vx_dot_kin = a
    # Kinematic v_y is the projection of v at angle beta onto body y.
    vy_kin = v_kin * ca.sin(beta)
    vy_dot_kin = (vy_kin - vy) / 0.1   # relax toward kinematic v_y over ~0.1 s
    r_kin = (v_kin / p.l_r) * ca.sin(beta)
    r_dot_kin = (r_kin - r) / 0.1      # relax toward kinematic r over ~0.1 s

    f_kin = ca.vertcat(
        vx * ca.cos(psi) - vy * ca.sin(psi),
        vx * ca.sin(psi) + vy * ca.cos(psi),
        r,
        vx_dot_kin,
        vy_dot_kin,
        r_dot_kin,
    )

    # Sigmoid blend: ~1.0 above v_min (dynamic), ~0.0 below (kinematic).
    sigma = 1.0 / (1.0 + ca.exp(-k * (vx - v_min)))
    f_expl = sigma * f_dyn + (1.0 - sigma) * f_kin
    return f_expl, x, u


# -----------------------------------------------------------------------------
# Quick self-test (sanity-check the math)
# -----------------------------------------------------------------------------


def steady_state_circle_test(p: BicycleParams, R: float = 50.0, v: float = 10.0
                              ) -> dict:
    """Predict steering angle on a steady-state circle for both models.

    On a circle of radius R at constant speed v, the kinematic bicycle predicts
    delta = atan2(L, R) and the dynamic bicycle predicts
    delta = L/R + K_us * v^2 / R (understeer gradient).

    Parameters
    ----------
    R : float
        Circle radius, m.
    v : float
        Constant speed, m/s.

    Returns
    -------
    dict
        Predicted deltas (rad) for both models plus K_us, R, and v, for
        unit-test comparison.
    """
    # Kinematic rear-axle form: tan(delta) = L / R.
    delta_kin = float(np.arctan2(p.L, R))

    K_us = (p.m / p.L) * (p.l_r / p.Cf - p.l_f / p.Cr)
    delta_dyn = float(p.L / R + K_us * v**2 / R)
    return {
        "delta_kin_rad": delta_kin,
        "delta_dyn_rad": delta_dyn,
        "K_us": K_us,
        "R": R,
        "v": v,
    }


def _main():
    ap = argparse.ArgumentParser(description="Self-test the vehicle models.")
    ap.add_argument("--L", type=float, default=2.85)
    ap.add_argument("--l_r", type=float, default=1.45)
    ap.add_argument("--R", type=float, default=50.0)
    ap.add_argument("--v", type=float, default=10.0)
    args = ap.parse_args()
    p = BicycleParams(L=args.L, l_r=args.l_r)

    for name, builder in [
        ("kinematic_bicycle_cg", kinematic_bicycle_cg),
        ("kinematic_bicycle_rear", kinematic_bicycle_rear),
        ("dynamic_bicycle_linear", dynamic_bicycle_linear),
    ]:
        f_expl, x, u = builder(p)
        print(f"{name}: nx={x.shape[0]}, nu={u.shape[0]}, f_expl shape={f_expl.shape}")

    res = steady_state_circle_test(p, R=args.R, v=args.v)
    print(f"\nSteady-state circle (R={args.R} m, v={args.v} m/s):")
    print(f"  delta_kin = {np.degrees(res['delta_kin_rad']):.3f} deg")
    print(f"  delta_dyn = {np.degrees(res['delta_dyn_rad']):.3f} deg")
    print(f"  K_us = {res['K_us']:.4e} rad/(m/s)^2")


if __name__ == "__main__":
    _main()
