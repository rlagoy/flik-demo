# Flik Demo

This repo demonstrates a pymavlink heartbeat and server control for the flik airframe.

## Installation

1. Install `flik-demo` and dependencies: `poetry install`
2. Install `pre-commit`: `poetry run pre-commit install`

## Contributions

1. Make a branch and pull request.
2. Ensure `pre-commit` runs successfully.
3. Run `pytest -q` to ensure all unit tests run successfully.
4. Run `pylint` to ensure code is up to python standards.


## Tools - Description

### Poetry

Poetry is a framework for managing dependencies.

#### Installation
This python template uses poetry for version control. You can learn more about poetry at the link below.

1. Install [poetry](https://python-poetry.org/docs/#installation).

#### Useful Commands
* To store a virtualenv in each project directory, run this command: `poetry config virtualenvs.in-project true`

### Gitignore

Adds a gitignore with common file types to ignore.

### Pre-commit

Example pre-commit config for python code. It runs a command to standardize/check code before commiting.

#### Install

1. `poetry run pre-commit install`
