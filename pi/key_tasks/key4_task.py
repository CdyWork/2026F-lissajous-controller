"""Matrix key 4: Q5 frequency lookup and zero-phase output."""


def run(*, run_tracking, run_measurement) -> None:
    del run_tracking
    return run_measurement(1)
