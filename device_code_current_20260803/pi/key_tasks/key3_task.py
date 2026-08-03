"""Matrix key 3: hard-lock tracking with a double-frequency output."""


def run(*, run_tracking, run_measurement) -> None:
    del run_measurement
    return run_tracking(3)
