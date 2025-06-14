import rox_control


def test_imports() -> None:
    """Test that the main modules can be imported without errors."""
    import rox_control
    import tools.bycicle_sim


def test_version() -> None:
    print(rox_control.__version__)
