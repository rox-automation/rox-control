# rox-control

Controllers for path following

## Development

### Local Development
```bash
# Setup environment
uv sync --all-extras

# Run tests
invoke test

# Lint and format code
invoke lint

# Create virtual environment
invoke create_venv
```

### VS Code DevContainer
Open in VS Code and select "Reopen in Container" for a pre-configured development environment with all tools and extensions.

## CI/CD

* **GitLab CI**: Trigger builds by tagging versions (see `.gitlab-ci.yml`)
* **Local CI**: Run `invoke ci` to test in Docker container

## Tooling

* **Dependencies**: `uv` for fast package management
* **Automation**: `invoke` - run `invoke -l` to list available commands
* **Versioning**: `setuptools_scm` with git tags
* **Linting**: `ruff` for fast linting and formatting
* **Type checking**: `mypy` for static analysis

## Project Structure

* `src/rox_control/` - Application code
* `tasks.py` - Automation tasks via invoke
* `pyproject.toml` - Modern Python packaging configuration