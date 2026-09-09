import pytest

from tools.lib.simulation_rendering import decimate_rendering


@pytest.mark.parametrize("interval", [1, 8])
def test_decimation_preserves_physics_steps_and_elapsed_simulation_time(interval):
    rendered = []
    physics_steps = []

    def physics_step(render=True):
        physics_steps.append(1 / 240)
        if render:
            rendered.append(len(physics_steps))
        return len(physics_steps)

    step = decimate_rendering(physics_step, interval)
    assert [step() for _ in range(240)] == list(range(1, 241))
    assert sum(physics_steps) == pytest.approx(1)
    assert rendered == list(range(interval, 241, interval))


def test_explicit_no_render_does_not_skip_physics_or_shift_cadence():
    calls = []
    step = decimate_rendering(lambda render: calls.append(render), 2)
    step()
    step(render=False)
    step(False)
    step()
    assert calls == [False, False, False, True]


@pytest.mark.parametrize("interval", [0, -1, 1.5, True])
def test_invalid_interval(interval):
    with pytest.raises(ValueError):
        decimate_rendering(lambda render: None, interval)
def test_live_pacing_preserves_steps_and_catches_up_between_expensive_redraws():
    from tools.lib.simulation_rendering import pace_live_rendering
    now = [0.0]
    rendered = []
    def step(render=True):
        rendered.append(render)
        now[0] += .0001 + (.03 if render else 0)
        return len(rendered)
    paced = pace_live_rendering(step, 1/240, clock=lambda: now[0])
    assert [paced() for _ in range(240)] == list(range(1,241))
    assert 25 <= sum(rendered) <= 40
    assert .9 <= now[0] <= 1.1


def test_live_pacing_never_forces_explicitly_disabled_rendering():
    from tools.lib.simulation_rendering import pace_live_rendering
    calls = []
    paced = pace_live_rendering(lambda render: calls.append(render), 1/240, clock=lambda:0)
    for _ in range(20):
        paced(render=False)
    assert calls == [False]*20


def test_live_pacing_discards_wall_clock_debt_after_a_pause():
    from tools.lib.simulation_rendering import pace_live_rendering
    now = [0.0]
    calls = []
    def step(render=True):
        calls.append(render)
        now[0] += 2 if len(calls)==1 else .03
    paced = pace_live_rendering(step, 1/240, clock=lambda:now[0])
    paced(); paced()
    assert calls == [True,True]
