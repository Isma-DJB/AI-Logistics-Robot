# Configuration

These YAML files contain the approved V1 baseline and hardware placeholders.

- `simulation.yaml` describes the 10 × 10 logical environment and reference scenario.
- `robot.yaml` reserves physical measurements and hardware settings.
- `logging.yaml` defines structured local logging.

`null` means that a value has not yet been measured or approved. It must not be replaced by a guessed physical value.

Machine-specific overrides belong in `configs/local/`, which is excluded from Git.
