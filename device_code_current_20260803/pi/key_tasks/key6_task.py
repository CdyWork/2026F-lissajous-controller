"""Matrix key 6: Q5 lookup followed by a double-frequency output."""


def run(*, run_tracking, run_measurement) -> None:
    del run_tracking
    return run_measurement(3)
