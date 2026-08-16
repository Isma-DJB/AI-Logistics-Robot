# Contributing to AI-Logistics-Robot

Thank you for your interest in AI-Logistics-Robot.

This repository is a simulation-first autonomous logistics robot project.
Contributions must preserve its deterministic behavior, platform-independent
core, architectural boundaries, and safety requirements.

## Project Status

The project is under active development.

Version 1 is implemented incrementally through reviewed implementation drafts.
Simulation behavior is completed before graphical and physical integration.

Before proposing a substantial change, review:

- `README.md`;
- `docs/requirements/v1/`;
- `docs/architecture/`;
- the latest record in `docs/implementation/`.

## Ways to Contribute

Useful contributions include:

- reporting reproducible defects;
- proposing focused improvements;
- improving tests and documentation;
- identifying safety, validation, or architectural problems;
- improving simulation and hardware-independent behavior.

Large features or architectural changes should be discussed in an issue before
implementation begins.

## Development Setup

AI-Logistics-Robot requires Python 3.11 or newer.

Create and activate a virtual environment, then install the project with its
development dependencies:

```powershell
py -3.11 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

On Linux or macOS:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

## Branch and Commit Workflow

Do not develop directly on `main`.

Create a focused branch from the current `main` branch. Examples include:

- `implementation/i-0.x`;
- `feature/short-description`;
- `fix/short-description`;
- `documentation/short-description`;
- `test/short-description`.

Keep commits focused and use clear messages such as:

```text
feat(planning): add deterministic path selection
fix(control): preserve pose after rejected movement
test(memory): verify reset lifecycle
docs: clarify simulation setup
```

Do not rewrite shared history or force-push changes to `main`.

## Architectural Boundaries

Contributions must preserve the dependency direction defined by the project:

- `domain` contains platform-independent immutable models and rules;
- `ports` contains short public contracts;
- core components depend on `domain` and `ports`;
- adapters implement external platform behavior;
- the Brain orchestrates but does not replace component responsibilities;
- rendering and monitoring must not influence control decisions.

Core modules must not directly import Pygame, hardware libraries, databases,
network clients, or platform-specific implementations.

Planning calculates paths but does not execute movement. Control executes one
safe step but does not orchestrate missions. Memory records confirmed facts but
does not make mission decisions.

## Code and Documentation Standards

Contributions must:

- use explicit type annotations;
- preserve deterministic behavior;
- validate public inputs;
- keep confirmed state immutable where required;
- include tests for new behavior and rejection rules;
- avoid unrelated formatting or refactoring changes;
- keep repository documentation in English;
- avoid credentials, tokens, personal data, generated secrets, and local
  environment files.

Generated artifacts, build outputs, virtual environments, caches, and local
configuration files must not be committed.

## Required Verification

Before opening a pull request, run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
python -m ruff check .
python -m mypy src
python tools/check_project_structure.py
python -m pip check
python -m ai_logistics_robot
git diff --check
```

If packaging behavior changes, also run:

```powershell
python -m build
```

Every reported failure must be resolved or clearly explained in the pull
request.

## Pull Requests

A pull request should:

- have one clear objective;
- describe the implemented behavior;
- identify affected requirements or architectural decisions;
- list the verification commands and results;
- document deferred concerns;
- contain no secrets or unrelated changes;
- keep the branch synchronized with the current `main` branch.

A pull request is ready for merge only when its changes, tests, documentation,
and architectural impact have been reviewed.

## Issues and Security Reports

Use the repository issue templates for reproducible bugs and feature proposals.

Do not disclose suspected vulnerabilities, credentials, or sensitive hardware
details in a public issue. Follow the instructions in `SECURITY.md` once the
security policy is published.

## Use of Automated or AI Assistance

Automated and AI-assisted tools may support research, drafting, implementation,
or review. The contributor remains responsible for:

- understanding every submitted change;
- verifying technical correctness;
- checking licensing and attribution;
- running the complete quality gate;
- ensuring that no confidential or sensitive information is submitted.

Unreviewed generated code or documentation should not be committed.

## Community Conduct

Participation in this project must remain respectful, constructive, and
professional. The repository Code of Conduct applies to all project
interactions.
