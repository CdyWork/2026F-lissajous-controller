"""Matrix key 1: hard-lock tracking with zero visual phase."""


def run(*, run_tracking, run_measurement) -> None:
    del run_measurement
    return run_tracking(1)
