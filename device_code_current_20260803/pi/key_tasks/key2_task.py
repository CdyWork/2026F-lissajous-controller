"""Matrix key 2: hard-lock tracking with a 90-degree output."""


def run(*, run_tracking, run_measurement) -> None:
    del run_measurement
    return run_tracking(2)
