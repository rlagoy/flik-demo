# Template Python Project

## Tools

### Poetry

Poetry is a framework for managing dependencies.

#### Installation
This python template uses poetry for version control. You can learn more about poetry at this [link]

1. Install [poetry](https://python-poetry.org/docs/#installation).

#### Useful Commands
* To store a virtualenv in each project directory, run this command: `poetry config virtualenvs.in-project true`

### Gitignore

Adds a gitignore with common file types to ignore

### Pre-commit

Example pre-commit config for python code. It runs a command to standardize/check code before commiting.

#### Install

1. `poetry run pre-commit install`

## Template Installation

1. Update `name`, `version`, `description`, and `authors` in the `pyproject.toml` file
2. Update the directory name to the module name you'd like.
3. Run `poetry install`.
