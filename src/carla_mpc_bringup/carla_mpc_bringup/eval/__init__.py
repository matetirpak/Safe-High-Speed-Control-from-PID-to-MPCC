"""Human-readable run evaluation: metrics, logs, and plots.

Tools for turning per-step control samples into reviewable artifacts, as
opposed to raw ROS streams.

metrics
    Pure functions mapping per-step series to a metrics dict, shared by the
    live logger and the benchmark aggregator.
run_logger
    RunLogger accumulates per-step samples during a run, then writes
    logs/<run>/ with metrics.json, series.csv, plots, and summary.txt.
"""
