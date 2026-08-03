"""Matrix key 5: Q5 lookup followed by a 90-degree output."""


def run(*, run_tracking, run_measurement) -> None:
    del run_tracking
    return run_measurement(2)
