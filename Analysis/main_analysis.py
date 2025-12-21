"""
Main analysis program: simple data-driven search for better leg parameters.

Approach:
- Random sampling around baseline p0
- Filter infeasible (ok==False)
- Score = height + streak (both as incentives)
- Select Top-K, then coordinate hill climbing
- Save CSV/JSON artifacts to Analysis/artifacts

Run:
  python Analysis/main_analysis.py --n-init 500 --top-k 50 --step-size 0.5 --rounds 6

"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
import random
import sys
import time
from typing import List, Dict, Tuple

# Ensure local import works regardless of CWD
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

try:
    from leg_interface import evaluate_params
except Exception as e:
    raise RuntimeError(f"Failed to import leg_interface: {e}")

ARTIFACTS_DIR = os.path.join(THIS_DIR, "artifacts")
os.makedirs(ARTIFACTS_DIR, exist_ok=True)

# Baseline parameters from MATLAB grid_search.m
P0 = [15.0, 50.0, 41.5, 55.8, 40.1, 39.3, 61.9, 36.7, 39.4, 49.0, 65.7, -38.0, -7.8]


def clamp_params(p: List[float]) -> List[float]:
    """Ensure physical plausibility: link lengths positive; node4 free.
    """
    out = list(p)
    for i in range(11):
        out[i] = max(out[i], 1e-6)
    # node4_x, node4_y unconstrained here
    return out


def score_metrics(m: Dict[str, float], height_weight: float, spi_weight: float, asi_weight: float) -> float:
    """Score = length + w_h*height - w_spi*pen_spi - w_asi*pen_asi.
    pen_spi = 1/(spi_margin+eps), pen_asi = 1/(asi_margin+eps).
    """
    eps = 1e-9
    length = m.get("length") if "length" in m else m.get("streak")
    height = m.get("height") if "height" in m else m.get("step_height")
    spi_margin = m.get("spi_margin", 0.0)
    asi_margin = m.get("asi_margin", 0.0)
    if length is None or height is None:
        return float("-inf")
    if not (math.isfinite(length) and math.isfinite(height)):
        return float("-inf")
    pen_spi = 1.0 / (spi_margin + eps) if math.isfinite(spi_margin) else 1e6
    pen_asi = 1.0 / (asi_margin + eps) if math.isfinite(asi_margin) else 1e6
    return float(length) + float(height_weight) * float(height) - float(spi_weight) * pen_spi - float(asi_weight) * pen_asi


def eval_cached(p: List[float], cache: Dict[Tuple[float, ...], Dict[str, float]], theta_step_deg: int, height_weight: float, spi_weight: float, asi_weight: float) -> Dict[str, float]:
    key = tuple(round(x, 6) for x in p)
    if key in cache:
        return cache[key]
    res = evaluate_params(list(p), theta_step_deg=theta_step_deg)
    # unify keys for downstream
    out = {
        "ok": bool(res.get("ok", False)),
        # prefer length; keep streak for backward compatibility
        "length": float(res.get("length", res.get("streak", 0.0))),
        "height": float(res.get("height", 0.0)),
        "spi_margin": float(res.get("spi_margin", 0.0)),
        "asi_margin": float(res.get("asi_margin", 0.0)),
        "score": 0.0,
    }
    out["score"] = score_metrics(out, height_weight, spi_weight, asi_weight)
    cache[key] = out
    return out


def random_offsets(n_dim: int, low: float, high: float) -> List[float]:
    return [random.uniform(low, high) for _ in range(n_dim)]


def initial_sampling(p0: List[float], n_init: int, low: float, high: float, theta_step_deg: int, seed: int, height_weight: float, spi_weight: float, asi_weight: float) -> List[Dict]:
    rnd = random.Random(seed)
    random.seed(seed)
    results: List[Dict] = []
    cache: Dict[Tuple[float, ...], Dict[str, float]] = {}
    for i in range(n_init):
        offs = [rnd.uniform(low, high) for _ in range(13)]
        p = clamp_params([p0[j] + offs[j] for j in range(13)])
        met = eval_cached(p, cache, theta_step_deg, height_weight, spi_weight, asi_weight)
        if met["ok"]:
            results.append({"p": p, "metrics": met})
        if (i + 1) % max(1, n_init // 10) == 0:
            print(f"[init] {i+1}/{n_init} sampled, feasible={sum(r['metrics']['ok'] for r in results)}")
    return results


def hill_climb(p: List[float], theta_step_deg: int, step_size: float, rounds: int, cache: Dict[Tuple[float, ...], Dict[str, float]], height_weight: float, spi_weight: float, asi_weight: float) -> Tuple[List[float], Dict[str, float]]:
    best_p = list(p)
    best_m = eval_cached(best_p, cache, theta_step_deg, height_weight, spi_weight, asi_weight)
    if not best_m["ok"]:
        return best_p, best_m
    for r in range(rounds):
        improved = False
        for j in range(13):
            for delta in (+step_size, -step_size):
                cand = list(best_p)
                cand[j] += delta
                cand = clamp_params(cand)
                met = eval_cached(cand, cache, theta_step_deg, height_weight, spi_weight, asi_weight)
                if met["ok"] and met["score"] > best_m["score"]:
                    best_p, best_m = cand, met
                    improved = True
        print(f"[hill] round {r+1}/{rounds}, score={best_m['score']:.4f}, length={best_m.get('length', best_m.get('streak', 0.0)):.4f}, height={best_m['height']:.4f}")
        if not improved:
            break
    return best_p, best_m


def save_csv(path: str, rows: List[Dict]):
    if not rows:
        return
    # unify headers
    headers = [
        "score", "ok", "length", "height", "spi_margin", "asi_margin",
        *[f"p{i+1}" for i in range(13)],
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=headers)
        w.writeheader()
        for r in rows:
            p = r["p"]
            m = r["metrics"]
            row = {
                "score": m.get("score"),
                "ok": m.get("ok"),
                "length": m.get("length", m.get("streak")),
                "height": m.get("height"),
                "spi_margin": m.get("spi_margin"),
                "asi_margin": m.get("asi_margin"),
            }
            for i in range(13):
                row[f"p{i+1}"] = p[i]
            w.writerow(row)


def _format_vec(vec: List[float], decimals: int = 3) -> str:
    fmt = f"{{:.{decimals}f}}"
    return "[" + ", ".join(fmt.format(x) for x in vec) + "]"


def main():
    ap = argparse.ArgumentParser(description="Simple data analysis search for better leg params")
    ap.add_argument("--n-init", type=int, default=600, help="initial random samples")
    ap.add_argument("--top-k", type=int, default=50, help="number of top candidates to refine")
    ap.add_argument("--low", type=float, default=-2.0, help="lower bound for random offsets")
    ap.add_argument("--high", type=float, default=2.0, help="upper bound for random offsets")
    ap.add_argument("--theta-coarse", type=int, default=20, help="theta step in degrees for coarse eval")
    ap.add_argument("--theta-fine", type=int, default=10, help="theta step in degrees for fine eval")
    ap.add_argument("--step-size", type=float, default=0.5, help="hill climb step size")
    ap.add_argument("--rounds", type=int, default=6, help="hill climb rounds")
    ap.add_argument("--seed", type=int, default=42, help="random seed")
    ap.add_argument("--height-weight", type=float, default=0.5, help="weight of height in score")
    ap.add_argument("--spi-weight", type=float, default=1.0, help="penalty weight for SPI (singularity proximity)")
    ap.add_argument("--asi-weight", type=float, default=1.0, help="penalty weight for ASI (angle safety margin)")
    ap.add_argument("--dry-run", action="store_true", help="evaluate baseline only and exit")
    args = ap.parse_args()

    t0 = time.time()

    if args.dry_run:
        res = evaluate_params(list(P0), theta_step_deg=args.theta_fine)
        print({k: (round(v, 6) if isinstance(v, float) else v) for k, v in res.items()})
        return

    print("== Initial random sampling ==")
    init = initial_sampling(P0, args.n_init, args.low, args.high, args.theta_coarse, args.seed, args.height_weight, args.spi_weight, args.asi_weight)
    init_sorted = sorted(init, key=lambda r: r["metrics"]["score"], reverse=True)
    save_csv(os.path.join(ARTIFACTS_DIR, "sampled_results.csv"), init_sorted)
    print(f"Saved sampled_results.csv ({len(init_sorted)} feasible)")

    top = init_sorted[: min(args.top_k, len(init_sorted))]
    print(f"== Refinement of Top-{len(top)} via hill climbing ==")
    cache: Dict[Tuple[float, ...], Dict[str, float]] = {}

    refined_rows: List[Dict] = []
    best_overall = None
    best_metrics = None

    for idx, r in enumerate(top, 1):
        bp, bm = hill_climb(r["p"], args.theta_fine, args.step_size, args.rounds, cache, args.height_weight, args.spi_weight, args.asi_weight)
        refined_rows.append({"p": bp, "metrics": bm})
        if bm.get("ok") and (best_metrics is None or bm["score"] > best_metrics["score"]):
            best_overall, best_metrics = bp, bm
        print(f"[refine] {idx}/{len(top)} done")

    refined_sorted = sorted(refined_rows, key=lambda r: r["metrics"]["score"], reverse=True)
    save_csv(os.path.join(ARTIFACTS_DIR, "refined_results.csv"), refined_sorted)
    print("Saved refined_results.csv")

    if best_overall is not None:
        best_path = os.path.join(ARTIFACTS_DIR, "best_params.json")
        with open(best_path, "w") as f:
            json.dump({
                "params": best_overall,
                "metrics": best_metrics,
            }, f, indent=2)
        print(f"Saved best_params.json: score={best_metrics['score']:.4f}, length={best_metrics.get('length', best_metrics.get('streak', 0.0)):.4f}, height={best_metrics['height']:.4f}")
        # Print MATLAB-friendly copy format: [links]与[node4]
        links = best_overall[:11]
        node4 = best_overall[11:13]
        print(_format_vec(links) + "与" + _format_vec(node4))

    # Record run statistics and arguments
    stats_path = os.path.join(ARTIFACTS_DIR, "statistics.json")
    run_stats = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        "args": {
            "n_init": args.n_init,
            "top_k": args.top_k,
            "low": args.low,
            "high": args.high,
            "theta_coarse": args.theta_coarse,
            "theta_fine": args.theta_fine,
            "step_size": args.step_size,
            "rounds": args.rounds,
            "seed": args.seed,
            "height_weight": args.height_weight,
            "spi_weight": args.spi_weight,
            "asi_weight": args.asi_weight,
        },
        "feasible_init": len(init_sorted),
        "refined_count": len(refined_sorted),
    }
    if best_overall is not None and best_metrics is not None:
        run_stats["best"] = {
            "score": best_metrics.get("score"),
            "length": best_metrics.get("length", best_metrics.get("streak")),
            "height": best_metrics.get("height"),
            "spi_margin": best_metrics.get("spi_margin"),
            "asi_margin": best_metrics.get("asi_margin"),
            "params_links": links,
            "params_node4": node4,
        }
    # Append to list in statistics.json
    try:
        if os.path.exists(stats_path):
            with open(stats_path, "r") as f:
                existing = json.load(f)
            if not isinstance(existing, list):
                existing = []
        else:
            existing = []
    except Exception:
        existing = []
    existing.append(run_stats)
    with open(stats_path, "w") as f:
        json.dump(existing, f, indent=2)

    # Also write a small best_examples.csv (top 10 refined)
    best_examples = refined_sorted[: min(10, len(refined_sorted))]
    save_csv(os.path.join(ARTIFACTS_DIR, "best_examples.csv"), best_examples)
    print("Saved best_examples.csv")

    print(f"Done in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
