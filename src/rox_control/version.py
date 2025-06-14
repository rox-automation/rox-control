from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("rox_control")
except PackageNotFoundError:  # pragma: no cover
    # Package is not installed, and therefore, version is unknown.
    __version__ = "0.0.0+unknown"
