# type: ignore
import os
import time

from invoke import task


@task
def clean(ctx):
    """
    Remove all files and directories that are not under version control to ensure a pristine working environment.
    Use caution as this operation cannot be undone and might remove untracked files.

    """

    ctx.run("git clean -nfdx")

    response = input("Are you sure you want to remove all untracked files? (y/n) [n]: ")
    if response.lower() in ("y", "yes"):
        ctx.run("git clean -fdx")


@task
def lint(ctx):
    """
    Perform static analysis on the source code to check for syntax errors and enforce style consistency.
    """
    ctx.run("uv run ruff check --fix src", pty=True)
    ctx.run("uv run ruff format src", pty=True)
    ctx.run("uv run mypy src", pty=True)


@task
def test(ctx):
    """
    Run tests with coverage reporting to ensure code functionality and quality.
    """
    ctx.run("uv run pytest --cov=src --cov-report term-missing tests", pty=True)


@task
def create_venv(ctx):
    """
    Create a virtual environment for the project.
    """
    ctx.run("uv sync --all-extras")


@task
def install(ctx):
    """
    Install the package in development mode with all dependencies.
    """
    ctx.run('uv pip install -e ".[dev]"')


@task
def uml(ctx):
    """
    Generate UML diagrams from the source code using pyreverse.
    """
    ctx.run("mkdir -p docs/uml")
    ctx.run("pyreverse src/rox_control -o png -d docs/uml")


@task
def ci(ctx):
    """
    run ci locally in a fresh container

    """
    t_start = time.time()
    # get script directory
    script_dir = os.path.dirname(os.path.realpath(__file__))
    try:
        # Build the CI image from project root using ci/Dockerfile
        ctx.run(f"docker build -t rox-control-ci -f {script_dir}/ci/Dockerfile {script_dir}")
        # Run CI in container with pre-installed venv
        ctx.run(f"docker run --rm -e UV_PROJECT_ENVIRONMENT=/app/.venv -v {script_dir}:/workspace rox-control-ci")
    finally:
        t_end = time.time()
        print(f"CI run took {t_end - t_start:.1f} seconds")


@task
def build_package(ctx):
    """
    Build package in docker container.
    """

    ctx.run("rm -rf dist")
    t_start = time.time()
    # get script directory
    script_dir = os.path.dirname(os.path.realpath(__file__))
    try:
        ctx.run(
            f"docker run --rm -v {script_dir}:/workspace roxauto/python-ci /scripts/build.sh"
        )
    finally:
        t_end = time.time()
        print(f"CI run took {t_end - t_start:.1f} seconds")


@task
def release(ctx):
    """publish package to pypi"""
    script_dir = os.path.dirname(os.path.realpath(__file__))

    token = os.getenv("PYPI_TOKEN")
    if not token:
        raise ValueError("PYPI_TOKEN environment variable is not set")

    ctx.run(
        f"docker run --rm -e PYPI_TOKEN={token} -v {script_dir}:/workspace roxauto/python-ci /scripts/publish.sh"
    )
