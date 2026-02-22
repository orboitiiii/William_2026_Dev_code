#!/usr/bin/env python3
"""
Optimal Fitting Function Generator for ShotTables
==================================================
Team 9427 — 2026

Analyzes the existing shot table data points and finds the best-fit
closed-form functions for each curve (Hood Angle, Flywheel Speed, ToF).

Approaches tried per curve:
  1. Natural Cubic Spline (piecewise, guaranteed to pass through every point)
  2. Rational functions: P(x)/Q(x)
  3. Exponential + polynomial hybrids
  4. Minimax (Chebyshev) polynomial

Outputs:
  - Comparison table of max/RMS residuals for each approach
  - Java double[] initializer code for the winner
  - Validation plots

Usage:
  python generate_spline_coefficients.py
"""

import math
import warnings

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import curve_fit, least_squares

warnings.filterwarnings("ignore")

# ============================================================================
# Raw data from ShotTables.java (15 nodes)
# ============================================================================
DISTANCES = np.array(
    [
        0.50,
        0.65,
        0.80,
        0.95,
        1.10,
        1.25,
        1.40,
        2.30,
        2.40,
        3.00,
        3.25,
        4.20,
        5.15,
        6.10,
        6.50,
    ]
)

HOOD_ANGLES = np.array(
    [
        1.448623,
        1.423455,
        1.398287,
        1.373119,
        1.347951,
        1.322783,
        1.297615,
        1.146607,
        1.129828,
        1.029156,
        1.031068,
        1.038334,
        1.045599,
        1.052865,
        1.055924,
    ]
)

FLYWHEEL_SPEEDS = np.array(
    [
        39.5075,
        40.4292,
        41.1517,
        41.7522,
        42.2778,
        42.7581,
        43.2124,
        45.9368,
        46.2712,
        48.5282,
        52.8332,
        57.7891,
        62.5810,
        67.1673,
        69.0400,
    ]
)

TIME_OF_FLIGHT = np.array(
    [
        0.747701,
        0.788496,
        0.815415,
        0.834165,
        0.847636,
        0.857454,
        0.864603,
        0.875877,
        0.874970,
        0.863366,
        0.861842,
        1.030780,
        1.181754,
        1.320766,
        1.376595,
    ]
)

# Dense evaluation grid for error analysis
D_DENSE = np.linspace(0.50, 6.50, 2000)


# ============================================================================
# Candidate function forms
# ============================================================================


# --- Hood Angle models ---
def hood_rational_22(d, a, b, c, e, f):
    """Rational function P2/Q2: (a + b*d + c*d²) / (1 + e*d + f*d²)"""
    return (a + b * d + c * d**2) / (1.0 + e * d + f * d**2)


def hood_exp_decay(d, a, b, c, e):
    """Exponential decay + linear: a + b*exp(-c*d) + e*d"""
    return a + b * np.exp(-c * d) + e * d


def hood_logistic(d, L, K, k, d0, s):
    """Generalized logistic/sigmoid: L + K / (1 + exp(k*(d-d0))) + s*d"""
    return L + K / (1.0 + np.exp(k * (d - d0))) + s * d


def hood_rational_32(d, a, b, c, g, e, f):
    """Rational function P3/Q2: (a + b*d + c*d² + g*d³) / (1 + e*d + f*d²)"""
    return (a + b * d + c * d**2 + g * d**3) / (1.0 + e * d + f * d**2)


# --- Flywheel Speed models ---
def fw_poly3(d, a, b, c, e):
    """Cubic polynomial"""
    return a + b * d + c * d**2 + e * d**3


def fw_rational_21(d, a, b, c, e):
    """Rational P2/Q1: (a + b*d + c*d²) / (1 + e*d)"""
    return (a + b * d + c * d**2) / (1.0 + e * d)


def fw_sqrt_poly(d, a, b, c, e):
    """sqrt + polynomial: a + b*sqrt(d) + c*d + e*d²"""
    return a + b * np.sqrt(d) + c * d + e * d**2


def fw_rational_22(d, a, b, c, e, f):
    """Rational P2/Q2"""
    return (a + b * d + c * d**2) / (1.0 + e * d + f * d**2)


# --- Time of Flight models ---
def tof_rational_32(d, a, b, c, g, e, f):
    """Rational P3/Q2"""
    return (a + b * d + c * d**2 + g * d**3) / (1.0 + e * d + f * d**2)


def tof_rational_33(d, a, b, c, g, e, f, h):
    """Rational P3/Q3"""
    return (a + b * d + c * d**2 + g * d**3) / (1.0 + e * d + f * d**2 + h * d**3)


def tof_poly4(d, a, b, c, e, f):
    """Quartic polynomial"""
    return a + b * d + c * d**2 + e * d**3 + f * d**4


def tof_poly5(d, a, b, c, e, f, g):
    """Quintic polynomial"""
    return a + b * d + c * d**2 + e * d**3 + f * d**4 + g * d**5


# ============================================================================
# Fitting engine
# ============================================================================
def try_fit(func, xdata, ydata, p0=None, bounds=(-np.inf, np.inf), name=""):
    """Try curve_fit with robust error handling."""
    n_params = func.__code__.co_argcount - 1  # subtract 'd' parameter
    if p0 is None:
        p0 = np.ones(n_params)
    try:
        popt, pcov = curve_fit(func, xdata, ydata, p0=p0, maxfev=50000, bounds=bounds)
        y_pred = func(xdata, *popt)
        residuals = ydata - y_pred
        max_err = np.max(np.abs(residuals))
        rms_err = np.sqrt(np.mean(residuals**2))

        # Evaluate on dense grid to check for oscillation/blowup
        y_dense = func(D_DENSE, *popt)
        y_range = np.max(ydata) - np.min(ydata)
        dense_range = np.max(y_dense) - np.min(y_dense)
        # If dense range is more than 3x the data range, it's overfitting/oscillating
        stable = dense_range < 3.0 * max(y_range, 0.01)

        return {
            "name": name,
            "params": popt,
            "max_err": max_err,
            "rms_err": rms_err,
            "n_params": len(popt),
            "stable": stable,
            "func": func,
        }
    except Exception as e:
        return {
            "name": name,
            "max_err": np.inf,
            "rms_err": np.inf,
            "stable": False,
            "error": str(e),
            "n_params": 0,
        }


def analyze_curve(name, xdata, ydata, candidates):
    """Analyze all candidate functions and rank them."""
    print(f"\n{'='*65}")
    print(f"  Analyzing: {name}")
    print(f"{'='*65}")

    results = []
    for cand in candidates:
        r = try_fit(
            cand["func"],
            xdata,
            ydata,
            p0=cand.get("p0"),
            bounds=cand.get("bounds", (-np.inf, np.inf)),
            name=cand["name"],
        )
        results.append(r)

    # Also compute cubic spline reference
    cs = CubicSpline(xdata, ydata, bc_type="natural")
    cs_pred = cs(xdata)
    cs_max = np.max(np.abs(ydata - cs_pred))
    cs_rms = np.sqrt(np.mean((ydata - cs_pred) ** 2))

    print(
        f"\n  {'Method':<30} {'Params':>6}  {'Max Err':>12}  {'RMS Err':>12}  {'OK':>6}"
    )
    print(f"  {'-'*30} {'-'*6}  {'-'*12}  {'-'*12}  {'-'*6}")
    print(
        f"  {'Cubic Spline (reference)':<30} {14*4:>6}  {cs_max:>12.2e}  {cs_rms:>12.2e}  {'Y':>6}"
    )

    for r in sorted(results, key=lambda x: x["max_err"]):
        stable_str = "Y" if r.get("stable", False) else "N"
        err_str = f'{r["max_err"]:.2e}' if r["max_err"] < np.inf else "FAIL"
        rms_str = f'{r["rms_err"]:.2e}' if r["rms_err"] < np.inf else "FAIL"
        print(
            f"  {r['name']:<30} {r['n_params']:>6}  {err_str:>12}  {rms_str:>12}  {stable_str:>6}"
        )

    # Find best stable closed-form
    stable_results = [
        r for r in results if r.get("stable", False) and r["max_err"] < np.inf
    ]
    if stable_results:
        best = min(stable_results, key=lambda x: x["max_err"])
        print(
            f"\n  ★ Best closed-form: {best['name']} (max_err={best['max_err']:.2e}, {best['n_params']} params)"
        )
    else:
        best = None
        print(f"\n  ⚠ No stable closed-form fit found, using cubic spline")

    return best, cs


# ============================================================================
# Generate Java code for different function types
# ============================================================================
def format_double(v, precision=15):
    """Format a double with enough precision for Java."""
    return f"{v:.{precision}e}"


def generate_spline_java(name_prefix, cs, knots):
    """Generate Java double[] arrays for a CubicSpline."""
    n = len(knots)
    coeffs = cs.c  # shape: (4, n-1) — [d³, d², d¹, d⁰] per segment

    lines = []
    lines.append(
        f"  // ── {name_prefix} Cubic Spline Coefficients ({n} knots, {n-1} segments) ──"
    )
    lines.append(f"  // f(x) = a + b*(x-xi) + c*(x-xi)^2 + d*(x-xi)^3")
    lines.append(f"  // Coefficients from scipy.interpolate.CubicSpline (natural BC)")

    # Knots
    knot_str = ", ".join(f"{k}" for k in knots)
    lines.append(
        f"  private static final double[] {name_prefix}_KNOTS = {{{knot_str}}};"
    )

    # a = constant term (= y value at each knot)
    a_vals = [cs(knots[i]) for i in range(n - 1)]
    a_str = ", ".join(format_double(v) for v in a_vals)
    lines.append(f"  private static final double[] {name_prefix}_A = {{{a_str}}};")

    # b = linear coefficient
    b_vals = coeffs[2, :]  # row 2
    b_str = ", ".join(format_double(v) for v in b_vals)
    lines.append(f"  private static final double[] {name_prefix}_B = {{{b_str}}};")

    # c = quadratic coefficient
    c_vals = coeffs[1, :]  # row 1
    c_str = ", ".join(format_double(v) for v in c_vals)
    lines.append(f"  private static final double[] {name_prefix}_C = {{{c_str}}};")

    # d = cubic coefficient
    d_vals = coeffs[0, :]  # row 0
    d_str = ", ".join(format_double(v) for v in d_vals)
    lines.append(f"  private static final double[] {name_prefix}_D = {{{d_str}}};")

    return lines


def generate_closedform_java(name, func_name, params, java_expr_template):
    """Generate a Java method for a closed-form function."""
    lines = []
    # Parameter constants
    for i, p in enumerate(params):
        lines.append(f"  private static final double {name}_P{i} = {format_double(p)};")
    lines.append(f"  // Java expression: {java_expr_template}")
    return lines


# ============================================================================
# Main analysis
# ============================================================================
def main():
    print("=" * 65)
    print("  Shot Table Fitting Analysis")
    print("  Team 9427 | 2026")
    print("=" * 65)

    # ── Hood Angle ──
    hood_candidates = [
        {
            "name": "Rational P2/Q2 (5p)",
            "func": hood_rational_22,
            "p0": [1.5, -0.2, 0.01, 0.1, 0.01],
        },
        {
            "name": "Rational P3/Q2 (6p)",
            "func": hood_rational_32,
            "p0": [1.5, -0.2, 0.01, 0.001, 0.1, 0.01],
        },
        {
            "name": "Exp decay + linear (4p)",
            "func": hood_exp_decay,
            "p0": [1.0, 0.5, 1.0, 0.001],
        },
        {
            "name": "Logistic + linear (5p)",
            "func": hood_logistic,
            "p0": [1.03, 0.42, 2.0, 1.5, 0.001],
        },
    ]
    hood_best, hood_cs = analyze_curve(
        "Hood Angle [rad]", DISTANCES, HOOD_ANGLES, hood_candidates
    )

    # ── Flywheel Speed ──
    fw_candidates = [
        {"name": "Cubic poly (4p)", "func": fw_poly3, "p0": [38, 2, 0.5, -0.01]},
        {
            "name": "Rational P2/Q1 (4p)",
            "func": fw_rational_21,
            "p0": [40, 5, 1, -0.01],
        },
        {"name": "sqrt + poly (4p)", "func": fw_sqrt_poly, "p0": [30, 10, 2, 0.5]},
        {
            "name": "Rational P2/Q2 (5p)",
            "func": fw_rational_22,
            "p0": [40, 5, 1, 0.01, 0.001],
        },
    ]
    fw_best, fw_cs = analyze_curve(
        "Flywheel Speed [rot/s]", DISTANCES, FLYWHEEL_SPEEDS, fw_candidates
    )

    # ── Time of Flight ──
    tof_candidates = [
        {
            "name": "Rational P3/Q2 (6p)",
            "func": tof_rational_32,
            "p0": [0.5, 0.5, -0.1, 0.01, 0.5, -0.05],
        },
        {
            "name": "Rational P3/Q3 (7p)",
            "func": tof_rational_33,
            "p0": [0.5, 0.5, -0.1, 0.01, 0.5, -0.05, 0.01],
        },
        {
            "name": "Quartic poly (5p)",
            "func": tof_poly4,
            "p0": [0.5, 0.5, -0.1, 0.01, 0.001],
        },
        {
            "name": "Quintic poly (6p)",
            "func": tof_poly5,
            "p0": [0.5, 0.5, -0.1, 0.01, 0.001, 0.0001],
        },
    ]
    tof_best, tof_cs = analyze_curve(
        "Time of Flight [s]", DISTANCES, TIME_OF_FLIGHT, tof_candidates
    )

    # ── Decision and Java output ──
    print(f"\n\n{'='*65}")
    print("  FINAL DECISION")
    print(f"{'='*65}")

    use_spline = {}
    for curve_name, best, cs in [
        ("hood", hood_best, hood_cs),
        ("flywheel", fw_best, fw_cs),
        ("tof", tof_best, tof_cs),
    ]:
        if best and best["max_err"] < 1e-4:
            print(
                f"  {curve_name}: Using {best['name']} (max_err={best['max_err']:.2e})"
            )
            use_spline[curve_name] = False
        elif best and best["max_err"] < 5e-3:
            print(
                f"  {curve_name}: Closed-form acceptable ({best['name']}, max_err={best['max_err']:.2e})"
            )
            print(
                f"           But cubic spline is exact — using SPLINE for max precision"
            )
            use_spline[curve_name] = True
        else:
            print(f"  {curve_name}: No good closed-form → using CUBIC SPLINE")
            use_spline[curve_name] = True

    # ── Generate Java code ──
    print(f"\n\n{'='*65}")
    print("  JAVA CODE OUTPUT")
    print(f"{'='*65}\n")

    all_java_lines = []

    # Always generate spline coefficients (we'll use them)
    for prefix, cs_obj, data_label in [
        ("HOOD_SPLINE", hood_cs, "Hood Angle"),
        ("FW_SPLINE", fw_cs, "Flywheel Speed"),
        ("TOF_SPLINE", tof_cs, "Time of Flight"),
    ]:
        java_lines = generate_spline_java(prefix, cs_obj, DISTANCES)
        all_java_lines.extend(java_lines)
        all_java_lines.append("")

    # Also generate closed-form if any are good enough
    for curve_name, best in [
        ("hood", hood_best),
        ("flywheel", fw_best),
        ("tof", tof_best),
    ]:
        if best and best["max_err"] < 5e-3:
            all_java_lines.append(
                f"  // ── {curve_name} closed-form alternative: {best['name']} ──"
            )
            all_java_lines.append(
                f"  // Max error: {best['max_err']:.2e}, RMS: {best['rms_err']:.2e}"
            )
            for i, p in enumerate(best["params"]):
                all_java_lines.append(
                    f"  // private static final double {curve_name.upper()}_CF_P{i} = {format_double(p)};"
                )
            all_java_lines.append("")

    print("\n".join(all_java_lines))

    # ── Write to file ──
    out_path = __file__.replace(
        "generate_spline_coefficients.py", "spline_java_output.txt"
    )
    with open(out_path, "w") as f:
        f.write("\n".join(all_java_lines))
    print(f"\n  [OK] Java code saved to {out_path}")

    # ── Print spline coefficients in compact form for direct use ──
    print(f"\n\n{'='*65}")
    print("  SPLINE COEFFICIENT SUMMARY")
    print(f"{'='*65}")
    for prefix, cs_obj in [("HOOD", hood_cs), ("FLYWHEEL", fw_cs), ("TOF", tof_cs)]:
        n = len(DISTANCES)
        print(f"\n  {prefix} ({n-1} segments):")
        for i in range(n - 1):
            a = cs_obj(DISTANCES[i])
            b = cs_obj.c[2, i]
            c = cs_obj.c[1, i]
            d = cs_obj.c[0, i]
            print(
                f"    [{i:2d}] x=[{DISTANCES[i]:.2f},{DISTANCES[i+1]:.2f}] "
                f"a={a:+.10f} b={b:+.10e} c={c:+.10e} d={d:+.10e}"
            )

    # Verify: evaluate spline at all knots and print residuals
    print(f"\n\n{'='*65}")
    print("  KNOT RESIDUAL VERIFICATION")
    print(f"{'='*65}")
    for prefix, cs_obj, orig in [
        ("HOOD", hood_cs, HOOD_ANGLES),
        ("FW", fw_cs, FLYWHEEL_SPEEDS),
        ("TOF", tof_cs, TIME_OF_FLIGHT),
    ]:
        residuals = orig - cs_obj(DISTANCES)
        print(f"  {prefix}: max |residual| = {np.max(np.abs(residuals)):.2e}")


if __name__ == "__main__":
    main()
