import rox_control


def test_imports() -> None:
    """Test that the main modules can be imported without errors."""
    import rox_control  # noqa: F401
    import rox_control.tools.bicicle_model  # noqa: F401


def test_version() -> None:
    print(rox_control.__version__)


def test_simulation() -> None:
    from rox_control.tools.simulation import SimulationData, SimulationState

    _ = SimulationState()
    _ = SimulationData(states=[], track=None)
