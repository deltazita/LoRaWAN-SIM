"""
Conceptualization: Dimitrios Zorbas
Implementation: Saida Tulebayeva, Dimitrios Zorbas, GPT-5.4

LICENSED UNDER GPL v2
"""


import argparse
import itertools
import json
import re
import tempfile
import subprocess
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import joblib
import pandas as pd
import xgboost  # noqa: F401  # Needed so joblib can deserialize XGBoost models.
from tabulate import tabulate

# -----------------------------------------------------------------------------
# Paths and defaults
# -----------------------------------------------------------------------------

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_PERL_SCRIPT = SCRIPT_DIR / "LoRaWAN.pl"
DEFAULT_MODEL_DIR = SCRIPT_DIR / "models"
DEFAULT_CLASSIFIER_MODEL = DEFAULT_MODEL_DIR / "xgb_classifier.pkl"
DEFAULT_ENERGY_MODEL = DEFAULT_MODEL_DIR / "xgb_energy_regressor.pkl"

TARGET_CHOICES = ("prr", "pdr", "energy")
TARGET_FRIENDLY_NAMES = {"prr": "PRR", "pdr": "PDR", "energy": "Energy"}
CLASS_LABELS = {
    "prr": ["Poor", "Fair", "Good", "Excellent"],
    "pdr": ["Poor", "Fair", "Good", "Excellent"],
}
CLASSIFIER_TARGET_INDEX = {"prr": 0, "pdr": 1}
CONTRADICTIONS: List[Tuple[str, str, str, str]] = []

DEFAULT_CONFIG = {
    "packets_per_hour": 12,
    "simulation_time": 5,
    "nodes": 1000,
    "gateways": 1,
    "terrain_side": 2000,
    "number_of_bands": 2,
    "rx2sf": 12,
    "with_ack": 0,
    "max_retr": 1,
    "pkt_size": 16,
    "adr": 1,
    "double_gws": 0,
}

CANONICAL_KEYS = [
    "packets_per_hour",
    "simulation_time",
    "nodes",
    "gateways",
    "terrain_side",
    "number_of_bands",
    "rx2sf",
    "with_ack",
    "max_retr",
    "pkt_size",
    "adr",
    "double_gws",
]

# Accepted aliases for user input.
ALIASES_TO_CANONICAL = {
    "num_bands": "number_of_bands",
    "bands": "number_of_bands",
    "confirmed": "with_ack",
    "ack": "with_ack",
    "packet_size": "pkt_size",
    "payload_size": "pkt_size",
    "packet_rate": "packets_per_hour",
    "sim_time": "simulation_time",
    "simulation_time_hours": "simulation_time",
    "max_retries": "max_retr",
    "adr_on": "adr",
}

# Some models may expect these alternative feature names.
MODEL_FEATURE_ALIASES = {
    "max_retries": "max_retr",
    "adr_on": "adr",
    "confirmed": "with_ack",
    "num_bands": "number_of_bands",
    "packet_size": "pkt_size",
}

# Friendly names only for canonical parameters.
FRIENDLY_PARAMETER_NAMES = {
    "gateways": "Gateways",
    "packets_per_hour": "Packets per Hour",
    "pkt_size": "Packet Size",
    "double_gws": "Double Gateways",
    "nodes": "Nodes",
    "number_of_bands": "Number of Bands",
    "with_ack": "Confirmed Traffic",
    "max_retr": "Maximum Retransmissions",
    "adr": "ADR (Tx Power)",
    "rx2sf": "RX2 SF",
    "simulation_time": "Simulation Time",
    "terrain_side": "Terrain Side",
}

VERBOSE = False
SIM_CACHE: Dict[Tuple[int, ...], Dict[str, Any]] = {}
MODEL_CACHE: Dict[str, Dict[str, Any]] = {}


class SimulationError(RuntimeError):
    """Raised when the simulator cannot be executed or parsed."""


class ModelError(RuntimeError):
    """Raised when a model or its expected features are unavailable."""


def vprint(msg: str) -> None:
    """Print only in verbose mode."""
    if VERBOSE:
        print(f"{msg}")


def label_for_class(class_labels: Sequence[str], class_idx: int) -> str:
    """Return a readable class label."""
    if 0 <= class_idx < len(class_labels):
        return str(class_labels[class_idx])
    return f"Class {class_idx}"


def canonical_param_name(param: str) -> str:
    """Map aliases to canonical parameter names for display."""
    return ALIASES_TO_CANONICAL.get(param, param)


def normalize_config(config: Dict[str, Any]) -> Dict[str, int]:
    """Normalize a user configuration to the canonical integer-valued form."""
    normalized = dict(config)

    for alias, canonical in ALIASES_TO_CANONICAL.items():
        if alias in normalized and canonical not in normalized:
            normalized[canonical] = normalized[alias]

    for key, value in DEFAULT_CONFIG.items():
        normalized.setdefault(key, value)

    out: Dict[str, int] = {}
    for key in CANONICAL_KEYS:
        if key not in normalized:
            raise ValueError(f"Missing required config key: {key}")
        out[key] = int(normalized[key])

    # Extra aliases that some models may use internally.
    out["max_retries"] = out["max_retr"]
    out["adr_on"] = out["adr"]
    out["confirmed"] = out["with_ack"]
    out["num_bands"] = out["number_of_bands"]
    out["packet_size"] = out["pkt_size"]
    return out


def config_cache_key(config: Dict[str, Any]) -> Tuple[int, ...]:
    """Build a stable cache key for a normalized configuration."""
    cfg = normalize_config(config)
    return tuple(cfg[key] for key in CANONICAL_KEYS)


def perl_json_from_config(config: Dict[str, Any]) -> Dict[str, int]:
    """Build the JSON object expected by the JSON-capable Perl simulator."""
    cfg = normalize_config(config)
    return {key: int(cfg[key]) for key in CANONICAL_KEYS}


def write_perl_json_config(config: Dict[str, Any], directory: Path) -> Path:
    """Write a temporary JSON config file for the Perl simulator."""
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".json",
        prefix="lorawan_config_",
        dir=str(directory),
        delete=False,
        encoding="utf-8",
    ) as handle:
        json.dump(perl_json_from_config(config), handle, indent=2, sort_keys=True)
        handle.write("\n")
        return Path(handle.name)


def parse_simulator_output(stdout: str) -> Dict[str, Any]:
    """Extract PRR, PDR, and energy from LoRaWAN.pl output."""
    patterns = {
        "energy_j": r"Avg node consumption\s*=\s*([0-9]*\.?[0-9]+)\s*J",
        "pdr": r"Packet Delivery Ratio\s*=\s*([0-9]*\.?[0-9]+)",
        "prr": r"Packet Reception Ratio\s*=\s*([0-9]*\.?[0-9]+)",
    }

    parsed: Dict[str, Any] = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, stdout)
        if not match:
            raise SimulationError(f"Could not parse '{key}' from LoRaWAN.pl output.")
        parsed[key] = float(match.group(1))
    return parsed


def run_lorawan_simulator(config: Dict[str, Any], perl_script: Optional[str] = None) -> Dict[str, Any]:
    """Run the Perl simulator once for the baseline configuration."""
    key = config_cache_key(config)
    if key in SIM_CACHE:
        vprint("Using cached simulator result.")
        return SIM_CACHE[key]

    script_path = Path(perl_script).expanduser().resolve() if perl_script else DEFAULT_PERL_SCRIPT
    if not script_path.exists():
        raise FileNotFoundError(f"Perl simulator not found: {script_path}")

    terrain_gen = script_path.parent / "generate_terrain.pl"
    if not terrain_gen.exists():
        raise FileNotFoundError(
            "Missing companion script 'generate_terrain.pl'. "
            f"Expected it next to {script_path.name} at {terrain_gen}"
        )

    tmp_dir = script_path.parent / "tmp"
    json_config_path = write_perl_json_config(config, tmp_dir)
    cmd = ["perl", str(script_path), "--json", str(json_config_path)]
    print(f"Running {script_path.name} once for baseline.")
    vprint("Command: " + " ".join(cmd))

    try:
        completed = subprocess.run(cmd, cwd=str(script_path.parent), capture_output=True, text=True, check=False)
    finally:
        try:
            json_config_path.unlink()
        except FileNotFoundError:
            pass
    if completed.returncode != 0:
        raise SimulationError(
            f"{script_path.name} failed with return code {completed.returncode}.\n"
            f"STDERR:\n{completed.stderr.strip()}\n\nSTDOUT:\n{completed.stdout.strip()}"
        )

    parsed = parse_simulator_output(completed.stdout)
    parsed["stdout"] = completed.stdout
    parsed["stderr"] = completed.stderr
    SIM_CACHE[key] = parsed
    return parsed


def resolve_model_path(path_str: Optional[str], default_path: Path) -> Path:
    """Resolve a model path, falling back to the default path."""
    return Path(path_str).expanduser().resolve() if path_str else default_path


def load_model_bundle(path_str: Optional[str], default_path: Path, cache_prefix: str, error_name: str) -> Dict[str, Any]:
    """Load a model and its feature names with caching."""
    path = resolve_model_path(path_str, default_path)
    cache_key = f"{cache_prefix}::{path}"
    if cache_key in MODEL_CACHE:
        return MODEL_CACHE[cache_key]
    if not path.exists():
        raise ModelError(f"{error_name} file not found: {path}")

    model = joblib.load(path)
    features = list(getattr(model, "feature_names_in_", []))
    if not features:
        raise ModelError(f"The {error_name.lower()} does not expose feature_names_in_.")

    bundle = {"model": model, "features": features}
    MODEL_CACHE[cache_key] = bundle
    return bundle


def load_classifier_model(path_str: Optional[str] = None) -> Dict[str, Any]:
    """Load the PRR/PDR classifier model bundle."""
    return load_model_bundle(path_str, DEFAULT_CLASSIFIER_MODEL, "classifier", "Classifier model")


def load_energy_regressor(path_str: Optional[str] = None) -> Dict[str, Any]:
    """Load the energy regressor bundle."""
    return load_model_bundle(path_str, DEFAULT_ENERGY_MODEL, "energy", "Energy regressor model")


def lower_only_values(current: int, candidates: List[int], floor_ratio: Optional[float] = None) -> List[int]:
    """Keep only lower candidate values, optionally bounded by a floor ratio."""
    min_allowed = 1
    if floor_ratio is not None:
        min_allowed = max(1, int(current * floor_ratio))
    return sorted({value for value in candidates if min_allowed <= value < current})


def config_for_energy_model(config: Dict[str, Any]) -> Dict[str, int]:
    """Force energy-model predictions to use a 1-hour configuration."""
    cfg = normalize_config(config)
    cfg["simulation_time"] = 1
    return cfg


def to_one_hour_equivalent_energy(energy_j: float, simulation_time_h: int) -> float:
    """Normalize simulated energy to a 1-hour basis."""
    if simulation_time_h <= 0:
        raise ValueError("simulation_time must be positive.")
    return float(energy_j) / float(simulation_time_h)


def get_param_spaces(config: Dict[str, Any], target: str) -> Dict[str, List[int]]:
    """Return the search space for each tunable parameter."""
    cfg = normalize_config(config)
    terrain_all = list(range(500, 5501, 500))

    if cfg["with_ack"] == 1:
        if target == "pdr":
            max_retr_values = sorted({value for value in [1, 2, 4, 8] if value != cfg["max_retr"]})
        else:
            max_retr_values = lower_only_values(cfg["max_retr"], [1, 2, 4, 8])
    else:
        max_retr_values = []

    return {
        "gateways": sorted({value for value in range(1, 20, 2) if value > cfg["gateways"]}),
        "packets_per_hour": lower_only_values(cfg["packets_per_hour"], list(range(1, 59, 3)), floor_ratio=0.50),
        "pkt_size": lower_only_values(cfg["pkt_size"], [16, 32, 48]),
        "double_gws": [1] if cfg["double_gws"] == 0 else [0],
        "nodes": lower_only_values(cfg["nodes"], list(range(100, 4901, 300)), floor_ratio=0.50),
        "terrain_side": lower_only_values(cfg["terrain_side"], terrain_all, floor_ratio=0.50),
        "rx2sf": lower_only_values(cfg["rx2sf"], [9, 12]) if cfg["with_ack"] == 1 else [],
        "adr": [1] if cfg["adr"] == 0 else [0],
        "number_of_bands": [2] if cfg["number_of_bands"] == 1 else [],
        "with_ack": [] if cfg["with_ack"] == 0 else [0],
        "max_retr": max_retr_values,
    }


def model_row_from_config(config: Dict[str, Any], features: Sequence[str]) -> pd.DataFrame:
    """Build a feature row for a model from a normalized config."""
    cfg = normalize_config(config)
    row_data: Dict[str, Any] = {}
    for feature in features:
        if feature in cfg:
            row_data[feature] = cfg[feature]
        elif feature in MODEL_FEATURE_ALIASES and MODEL_FEATURE_ALIASES[feature] in cfg:
            row_data[feature] = cfg[MODEL_FEATURE_ALIASES[feature]]

    missing = [feature for feature in features if feature not in row_data]
    if missing:
        raise ModelError(f"Configuration is missing model feature(s): {', '.join(missing)}")

    return pd.DataFrame([row_data])[list(features)].astype(float)


def predict_classifier_details(config: Dict[str, Any], target: str, classifier_model_path: Optional[str] = None) -> Dict[str, Any]:
    """Predict PRR/PDR class details from the classifier."""
    if target not in CLASSIFIER_TARGET_INDEX:
        raise ModelError(f"Classification target must be 'prr' or 'pdr', got '{target}'.")

    bundle = load_classifier_model(classifier_model_path)
    row = model_row_from_config(config, bundle["features"])
    model = bundle["model"]
    target_index = CLASSIFIER_TARGET_INDEX[target]

    preds = model.predict(row)
    pred = preds[0]
    class_idx = int(pred[target_index]) if hasattr(pred, "__len__") and not isinstance(pred, (str, bytes)) else int(pred)

    probabilities = None
    if hasattr(model, "predict_proba"):
        try:
            raw = model.predict_proba(row)
            if isinstance(raw, list) and 0 <= target_index < len(raw):
                probabilities = [float(x) for x in raw[target_index][0].tolist()]
        except Exception:
            probabilities = None

    expected_class = None
    if probabilities:
        expected_class = sum(idx * prob for idx, prob in enumerate(probabilities))

    return {"class_idx": class_idx, "probabilities": probabilities, "expected_class": expected_class}


def predict_energy(config: Dict[str, Any], energy_model_path: Optional[str] = None) -> float:
    """Predict 1-hour energy from the regression model."""
    bundle = load_energy_regressor(energy_model_path)
    row = model_row_from_config(config_for_energy_model(config), bundle["features"])
    pred = bundle["model"].predict(row)
    if hasattr(pred, "tolist"):
        pred = pred.tolist()
    return float(pred[0] if isinstance(pred, list) else pred)


def change_signature(original: Dict[str, Any], modified: Dict[str, Any], target: str) -> Tuple[Tuple[str, str], ...]:
    """Encode only the direction of each parameter change."""
    sig = []
    for param in sorted(get_param_spaces(original, target).keys()):
        orig = original[param]
        new = modified[param]
        if orig == new:
            continue
        sig.append((param, "increase" if new > orig else "decrease"))
    return tuple(sig)


def is_contradictory(sig: Tuple[Tuple[str, str], ...]) -> bool:
    """Check whether a signature contains explicitly forbidden pairs."""
    sig_set = set(sig)
    for p_a, d_a, p_b, d_b in CONTRADICTIONS:
        if (p_a, d_a) in sig_set and (p_b, d_b) in sig_set:
            return True
    return False


def is_dominated(sig: Tuple[Tuple[str, str], ...], simpler_sigs: set) -> bool:
    """Drop complex signatures when a simpler accepted one already subsumes them."""
    sig_set = set(sig)
    for simpler in simpler_sigs:
        if set(simpler).issubset(sig_set) and len(simpler) < len(sig):
            return True
    return False


def score_candidate(original: Dict[str, Any], modified: Dict[str, Any], base_score: float, target: str) -> float:
    """Generic score: reward improvement and penalize large/many changes."""
    spaces = get_param_spaces(original, target)
    n_changes = sum(1 for p in spaces if modified[p] != original[p])
    total_delta = sum(abs(modified[p] - original[p]) for p in spaces if modified[p] != original[p])
    scale = 1000.0 if target == "energy" else 100.0
    return base_score * scale - n_changes * 10.0 - total_delta * 0.1


def _display_value(param: str, value: Any) -> str:
    """Format one parameter value for human-readable output."""
    value = int(value)
    if param == "with_ack":
        return "Enabled" if value == 1 else "Disabled"
    if param == "double_gws":
        return "Enabled" if value == 1 else "Disabled"
    if param in {"adr", "adr_on"}:
        return "Enabled" if value == 1 else "Disabled"
    if param == "number_of_bands":
        return f"{value} band" if value == 1 else f"{value} bands"
    if param == "pkt_size":
        return f"{value} bytes"
    if param == "simulation_time":
        return f"{value} h"
    if param == "terrain_side":
        return f"{value} m"
    return str(value)


def _friendly_change_text(param: str, old: Any, new: Any) -> str:
    """Format a single parameter change."""
    canonical = canonical_param_name(param)
    label = FRIENDLY_PARAMETER_NAMES.get(canonical, canonical)
    return f"{label}: {_display_value(canonical, old)} -> {_display_value(canonical, new)}"


def format_changes_single_line(changes: List[Tuple[str, Any, Any]]) -> str:
    """Format a list of parameter changes on one line."""
    if not changes:
        return "-"
    return "; ".join(_friendly_change_text(param, old, new) for param, old, new in changes)


def format_quality_effect(baseline_class: int, predicted_class: int, labels: Sequence[str]) -> str:
    """Describe whether a quality class increases, decreases, or stays the same."""
    baseline_label = label_for_class(labels, baseline_class)
    predicted_label = label_for_class(labels, predicted_class)
    if predicted_class > baseline_class:
        return f"Increase ({baseline_label} -> {predicted_label})"
    if predicted_class < baseline_class:
        return f"Decrease ({baseline_label} -> {predicted_label})"
    return "Unchanged"


def modified_config_key(config: Dict[str, Any], target: str) -> Tuple[int, ...]:
    """Build a key using only the parameters explored for the selected target."""
    cfg = normalize_config(config)
    search_keys = sorted(get_param_spaces(cfg, target).keys())
    return tuple(cfg[key] for key in search_keys)


def exhaustive_search(
    config: Dict[str, Any],
    target_class: Optional[int],
    target: str,
    classifier_model_path: Optional[str] = None,
    energy_model_path: Optional[str] = None,
    max_params: int = 3,
    baseline_energy_j: Optional[float] = None,
) -> List[Dict[str, Any]]:
    """Search all 1..max_params combinations and keep acceptable candidates."""
    spaces = get_param_spaces(config, target)
    tunable = [param for param, values in spaces.items() if values]
    candidates: List[Dict[str, Any]] = []
    winning_single_sigs = set()
    base_cfg = normalize_config(config)
    checked = 0

    for n_changes in range(1, max_params + 1):
        vprint(f"Exploring combinations with {n_changes} changed parameter(s).")
        for param_combo in itertools.combinations(tunable, n_changes):
            for values in itertools.product(*[spaces[p] for p in param_combo]):
                modified = base_cfg.copy()
                if all(values[i] == modified[param_combo[i]] for i in range(n_changes)):
                    continue

                for param, value in zip(param_combo, values):
                    modified[param] = value

                sig = change_signature(base_cfg, modified, target)
                if not sig or is_contradictory(sig):
                    continue
                checked += 1

                if target == "energy":
                    if baseline_energy_j is None:
                        raise ModelError("Missing baseline simulated energy for energy search.")
                    predicted_energy_j = predict_energy(modified, energy_model_path)
                    if predicted_energy_j >= baseline_energy_j - 1e-12:
                        continue
                    base_score = baseline_energy_j - predicted_energy_j
                    pred_value = predicted_energy_j
                else:
                    details = predict_classifier_details(modified, target, classifier_model_path)
                    predicted_class = int(details["class_idx"])
                    if target_class is None or predicted_class < target_class:
                        continue
                    base_score = float(predicted_class)
                    pred_value = predicted_class

                if is_dominated(sig, winning_single_sigs):
                    continue

                structured_changes = [
                    (param, base_cfg[param], modified[param])
                    for param in param_combo
                    if modified[param] != base_cfg[param]
                ]
                candidates.append(
                    {
                        "config": modified,
                        "config_key": modified_config_key(modified, target),
                        "signature": sig,
                        "score": score_candidate(base_cfg, modified, base_score, target),
                        "changes": structured_changes,
                        "changes_formatted": format_changes_single_line(structured_changes),
                        "pred": pred_value,
                    }
                )

                if n_changes == 1:
                    winning_single_sigs.add(sig)

    vprint(f"Checked {checked} candidate configuration(s); accepted {len(candidates)}.")
    return candidates


def deduplicate_recommendations(candidates: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Deduplicate first by final config, then by readable change text."""
    best_by_config: Dict[Tuple[int, ...], Dict[str, Any]] = {}
    for candidate in candidates:
        key = candidate["config_key"]
        existing = best_by_config.get(key)
        if existing is None or candidate["score"] > existing["score"]:
            best_by_config[key] = candidate

    best_by_text: Dict[str, Dict[str, Any]] = {}
    for candidate in best_by_config.values():
        key = candidate["changes_formatted"]
        existing = best_by_text.get(key)
        if existing is None or candidate["score"] > existing["score"]:
            best_by_text[key] = candidate
    return list(best_by_text.values())


def keep_best_by_signature(candidates: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Keep the single best candidate for each direction-only signature."""
    best_by_signature: Dict[Tuple[Tuple[str, str], ...], Dict[str, Any]] = {}
    for candidate in candidates:
        sig = candidate["signature"]
        existing = best_by_signature.get(sig)
        if existing is None or candidate["score"] > existing["score"]:
            best_by_signature[sig] = candidate
    return list(best_by_signature.values())


def run_recommendation(
    config: Dict[str, Any],
    top_n: int = 30,
    perl_script: Optional[str] = None,
    classifier_model: Optional[str] = None,
    energy_model: Optional[str] = None,
    target: str = "prr",
    max_params: int = 3,
    show_baseline_output: bool = False,
) -> List[Dict[str, Any]]:
    """Run the simulator once, then generate recommendations with the models."""
    cfg = normalize_config(config)
    target = target.lower()
    target_name = TARGET_FRIENDLY_NAMES[target]

    vprint(f"Using Perl script: {Path(perl_script).expanduser().resolve() if perl_script else DEFAULT_PERL_SCRIPT}")
    vprint(f"Using classifier model: {resolve_model_path(classifier_model, DEFAULT_CLASSIFIER_MODEL)}")
    vprint(f"Using energy regressor: {resolve_model_path(energy_model, DEFAULT_ENERGY_MODEL)}")
    vprint(f"Target: {target_name}")
    vprint(f"Baseline configuration: {json.dumps(cfg, sort_keys=True)}")

    baseline = run_lorawan_simulator(cfg, perl_script=perl_script)
    recommendation_start = time.perf_counter()

    print(f"Target : {target_name}")
    print(
        "Current metrics     : "
        f"PRR={baseline['prr']:.5f}, "
        f"PDR={baseline['pdr']:.5f}, "
        f"Energy={baseline['energy_j']:.5f} J"
    )
    if show_baseline_output:
        print("\n--- Baseline simulator output ---")
        print(baseline["stdout"])

    all_recommendations: List[Dict[str, Any]] = []
    baseline_predicted_energy_j = None if target == "energy" else predict_energy(cfg, energy_model)

    if target == "energy":
        baseline_energy_per_hour_j = to_one_hour_equivalent_energy(float(baseline["energy_j"]), cfg["simulation_time"])
        predicted_baseline_energy_j = predict_energy(cfg, energy_model)
        baseline_prr_class = int(predict_classifier_details(cfg, "prr", classifier_model)["class_idx"])
        baseline_pdr_class = None
        if cfg["with_ack"] == 1:
            baseline_pdr_class = int(predict_classifier_details(cfg, "pdr", classifier_model)["class_idx"])

        print(f"Baseline simulated energy (1h normalized) : {baseline_energy_per_hour_j:.5f} J")
        print(f"Baseline predicted energy (1h)           : {predicted_baseline_energy_j:.5f} J")
        print("\nSearching for configurations with lower predicted energy consumption...")

        candidates = exhaustive_search(
            cfg,
            target_class=None,
            target="energy",
            classifier_model_path=classifier_model,
            energy_model_path=energy_model,
            max_params=max_params,
            baseline_energy_j=baseline_energy_per_hour_j,
        )
        if not candidates:
            elapsed = time.perf_counter() - recommendation_start
            print("\nNo configurations were found with lower predicted energy consumption.")
            print(f"Recommendation system execution time (excluding simulation): {elapsed:.4f} sec")
            return []

        deduped = deduplicate_recommendations(keep_best_by_signature(candidates))
        top = sorted(deduped, key=lambda item: item["score"], reverse=True)[:top_n]

        for rec in top:
            display_score_j = predicted_baseline_energy_j - float(rec["pred"])
            rec_prr_class = int(predict_classifier_details(rec["config"], "prr", classifier_model)["class_idx"])
            prr_effect = format_quality_effect(baseline_prr_class, rec_prr_class, CLASS_LABELS["prr"])

            pdr_effect = None
            if baseline_pdr_class is not None:
                rec_pdr_class = int(predict_classifier_details(rec["config"], "pdr", classifier_model)["class_idx"])
                pdr_effect = format_quality_effect(baseline_pdr_class, rec_pdr_class, CLASS_LABELS["pdr"])

            all_recommendations.append(
                {
                    "target_quality": "Lower energy",
                    "changes_formatted": rec["changes_formatted"],
                    "score": display_score_j,
                    "config_key": rec["config_key"],
                    "prr_effect": prr_effect,
                    "pdr_effect": pdr_effect,
                }
            )
    else:
        current_details = predict_classifier_details(cfg, target, classifier_model)
        current_class = int(current_details["class_idx"])
        labels = CLASS_LABELS[target]
        max_class = len(labels) - 1
        print(f"Current class       : {label_for_class(labels, current_class)} (class {current_class})")

        if current_class >= max_class:
            elapsed = time.perf_counter() - recommendation_start
            print(f"Already at the highest available {target_name} class — nothing to recommend.")
            print(f"Recommendation system execution time (excluding simulation): {elapsed:.4f} sec")
            return []

        desired_classes = range(max_class, current_class, -1)
        for target_class in desired_classes:
            desired_label = label_for_class(labels, target_class)
            print(f"\nSearching for target: {desired_label} (class {target_class})...")

            candidates = exhaustive_search(
                cfg,
                target_class=target_class,
                target=target,
                classifier_model_path=classifier_model,
                energy_model_path=energy_model,
                max_params=max_params,
            )
            if not candidates:
                print(f"   No recommendations found for {desired_label}.")
                continue

            deduped = deduplicate_recommendations(keep_best_by_signature(candidates))
            top = sorted(deduped, key=lambda item: item["score"], reverse=True)[:top_n]

            for rec in top:
                predicted_energy_j = predict_energy(rec["config"], energy_model)
                energy_delta_j = predicted_energy_j - float(baseline_predicted_energy_j)
                if energy_delta_j < -1e-12:
                    energy_effect = f"Decrease ({abs(energy_delta_j):.5f} J)"
                elif energy_delta_j > 1e-12:
                    energy_effect = f"Increase ({energy_delta_j:.5f} J)"
                else:
                    energy_effect = "Unchanged"

                all_recommendations.append(
                    {
                        "target_quality": desired_label,
                        "changes_formatted": rec["changes_formatted"],
                        "predicted_quality": label_for_class(labels, int(rec["pred"])),
                        "score": rec["score"],
                        "config_key": rec["config_key"],
                        "energy_effect": energy_effect,
                    }
                )

            if len(top) >= top_n:
                print(f"   Found {len(top)} recommendations — stopping cascade.")
                break

    if not all_recommendations:
        elapsed = time.perf_counter() - recommendation_start
        print("\nNo recommendations found.")
        print(f"Recommendation system execution time (excluding simulation): {elapsed:.4f} sec")
        return []

    final_best: Dict[Tuple[int, ...], Dict[str, Any]] = {}
    for rec in all_recommendations:
        key = rec["config_key"]
        existing = final_best.get(key)
        if existing is None or rec["score"] > existing["score"]:
            final_best[key] = rec

    final_by_text: Dict[str, Dict[str, Any]] = {}
    for rec in final_best.values():
        key = rec["changes_formatted"]
        existing = final_by_text.get(key)
        if existing is None or rec["score"] > existing["score"]:
            final_by_text[key] = rec

    final_recommendations = sorted(final_by_text.values(), key=lambda item: item["score"], reverse=True)

    print("\n" + "=" * 100)
    print(f"RECOMMENDATIONS for {target_name} (unique, top recommendations)")
    print("=" * 100)

    rows = []
    if target == "energy":
        if cfg["with_ack"] == 1:
            for idx, rec in enumerate(final_recommendations, 1):
                rows.append([idx, rec["changes_formatted"], rec.get("prr_effect", "Unchanged"), rec.get("pdr_effect", "Unchanged"), f"{rec['score']:.5f} J"])
            headers = ["#", "Changes", "PRR Effect", "PDR Effect", "Score"]
            colalign = ("center", "left", "left", "left", "right")
        else:
            for idx, rec in enumerate(final_recommendations, 1):
                rows.append([idx, rec["changes_formatted"], rec.get("prr_effect", "Unchanged"), f"{rec['score']:.5f} J"])
            headers = ["#", "Changes", "PRR Effect", "Score"]
            colalign = ("center", "left", "left", "right")
    else:
        for idx, rec in enumerate(final_recommendations, 1):
            rows.append([idx, rec["changes_formatted"], rec["predicted_quality"], rec.get("energy_effect", "Unchanged"), f"{rec['score']:.1f}"])
        headers = ["#", "Changes", f"Predicted {target_name} Class", "Energy Effect", "Score"]
        colalign = ("center", "left", "center", "left", "right")

    print(tabulate(rows, headers=headers, tablefmt="grid", colalign=colalign))

    # print("\nSummary:")
    # target_counts = Counter(rec["target_quality"] for rec in final_recommendations)
    # for quality, count in sorted(target_counts.items(), key=lambda item: item[1], reverse=True):
    #     suffix = "s" if count != 1 else ""
    #     print(f"   - {quality}: {count} recommendation{suffix}")

    elapsed = time.perf_counter() - recommendation_start
    print(f"\nTotal unique recommendations found: {len(final_recommendations)}")
    print(f"Recommendation system execution time: {elapsed:.4f} sec")
    return final_recommendations


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run the JSON-capable LoRaWAN Perl simulator once for the baseline configuration, then use the classifier for PRR/PDR and the regressor for energy recommendations."
    )
    parser.add_argument("--config", type=str, default="", help="JSON configuration string. Missing fields fall back to defaults.")
    parser.add_argument("--config-file", type=str, default="", help="Path to a JSON file with the configuration.")
    parser.add_argument("--perl-script", type=str, default=str(DEFAULT_PERL_SCRIPT), help="Path to the JSON-capable Perl simulator. Defaults to ./LoRaWAN_json.pl next to this script.")
    parser.add_argument("--classifier-model", type=str, default=str(DEFAULT_CLASSIFIER_MODEL), help="Path to the PRR/PDR classifier model. Defaults to ./ML/xgb_classifier.pkl.")
    parser.add_argument("--energy-model", type=str, default=str(DEFAULT_ENERGY_MODEL), help="Path to the energy regressor model. Defaults to ./ML/xgb_energy_regressor.pkl.")
    parser.add_argument("--target", type=str, choices=TARGET_CHOICES, default="prr", help="Target: prr, pdr, or energy.")
    parser.add_argument("--top-n", type=int, default=30, help="Maximum number of recommendations to display. Default: 30.")
    parser.add_argument("--max-params", type=int, default=3, help="Maximum number of parameters to change jointly during search.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Show progress steps and internal search progress.")
    parser.add_argument("--show-baseline-output", action="store_true", help="Print the full stdout of the single baseline LoRaWAN.pl run.")
    return parser.parse_args()


def load_config_from_args(args: argparse.Namespace) -> Dict[str, int]:
    """Load configuration from --config-file and/or --config."""
    config_data: Dict[str, Any] = {}
    if args.config_file:
        config_path = Path(args.config_file).expanduser().resolve()
        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        vprint(f"Loading configuration file: {config_path}")
        with open(config_path, "r", encoding="utf-8") as handle:
            config_data.update(json.load(handle))

    if args.config:
        vprint("Loading configuration from --config JSON string.")
        config_data.update(json.loads(args.config))

    if not config_data:
        vprint("No external configuration provided; using built-in defaults.")
        config_data = dict(DEFAULT_CONFIG)

    return normalize_config(config_data)


def main() -> None:
    global VERBOSE
    args = parse_args()
    VERBOSE = args.verbose
    config = load_config_from_args(args)
    print("Recommendation engine ready.")
    run_recommendation(
        config=config,
        top_n=args.top_n,
        perl_script=args.perl_script,
        classifier_model=args.classifier_model,
        energy_model=args.energy_model,
        target=args.target,
        max_params=args.max_params,
        show_baseline_output=args.show_baseline_output,
    )


if __name__ == "__main__":
    main()
