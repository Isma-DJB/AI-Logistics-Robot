"""Validate the I-0.1 repository structure and dependency boundaries."""

from __future__ import annotations

import ast
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = PROJECT_ROOT / "src" / "ai_logistics_robot"

EXPECTED_DIRECTORIES = (
    "configs",
    "docs/architecture/diagrams",
    "docs/implementation",
    "docs/requirements/v1",
    "src/ai_logistics_robot/app",
    "src/ai_logistics_robot/domain",
    "src/ai_logistics_robot/ports",
    "src/ai_logistics_robot/brain",
    "src/ai_logistics_robot/planning",
    "src/ai_logistics_robot/perception",
    "src/ai_logistics_robot/control",
    "src/ai_logistics_robot/memory",
    "src/ai_logistics_robot/adapters/simulation",
    "src/ai_logistics_robot/adapters/visualization",
    "src/ai_logistics_robot/adapters/monitoring",
    "src/ai_logistics_robot/adapters/hardware",
    "tests/unit",
    "tests/integration/simulation",
    "tests/scenarios",
    "tests/hardware",
    "tools",
)

EXPECTED_FILES = (
    ".env.example",
    ".gitignore",
    "README.md",
    "pyproject.toml",
    "configs/logging.yaml",
    "configs/robot.yaml",
    "configs/simulation.yaml",
    "src/ai_logistics_robot/__init__.py",
    "src/ai_logistics_robot/__main__.py",
)

CORE_PACKAGES = (
    "domain",
    "ports",
    "brain",
    "planning",
    "perception",
    "control",
    "memory",
)

FORBIDDEN_IMPORT_ROOTS = {
    "arduino",
    "cv2",
    "esp32",
    "pygame",
    "serial",
}


def imported_roots(path: Path) -> set[str]:
    """Return the root names imported by one Python source file."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.partition(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.partition(".")[0])
    return roots


def collect_errors() -> list[str]:
    """Collect structural errors without changing the project."""
    errors: list[str] = []

    for relative in EXPECTED_DIRECTORIES:
        if not (PROJECT_ROOT / relative).is_dir():
            errors.append(f"missing directory: {relative}")

    for relative in EXPECTED_FILES:
        if not (PROJECT_ROOT / relative).is_file():
            errors.append(f"missing file: {relative}")

    for package_name in CORE_PACKAGES:
        package_path = PACKAGE_ROOT / package_name
        for source_path in package_path.rglob("*.py"):
            forbidden = imported_roots(source_path) & FORBIDDEN_IMPORT_ROOTS
            if forbidden:
                names = ", ".join(sorted(forbidden))
                errors.append(
                    f"forbidden platform import in {source_path.relative_to(PROJECT_ROOT)}: {names}"
                )

    return errors


def main() -> int:
    """Run the structural check and return a process exit code."""
    errors = collect_errors()
    if errors:
        print("I-0.1 structure check: FAILED")
        for error in errors:
            print(f"- {error}")
        return 1

    print("I-0.1 structure check: PASSED")
    print(f"Validated {len(EXPECTED_DIRECTORIES)} directories and {len(EXPECTED_FILES)} files.")
    print("Core packages contain no direct platform-specific imports.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
