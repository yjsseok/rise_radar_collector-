# Repository Guidelines

## Project Structure & Module Organization
- `sensr_lidar_recorder/` holds the Python recorders (`main.py`, `simple_pointcloud_recorder.py`), shared utilities under `src/`, and configuration assets in `config/`.
- `ros_test/` packages the ROS 2 driver; launch files sit in `src/sensr_ros2_driver/launch/` and container tooling lives beside the top-level `Dockerfile`.
- `sensr_ros_interface/` defines the ROS message schema in `msg/` along with the ROS 1 build descriptors (`CMakeLists.txt`, `package.xml`).
- Repository-wide reference material (API specs, troubleshooting, quick starts) is cached in `*.md` at the root; keep new guides there unless they are code-specific.

## Build, Test, and Development Commands
- `python -m venv .venv && .\.venv\Scripts\Activate.ps1` (Windows) or `source .venv/bin/activate` (Unix) to create an isolated environment before installing dependencies.
- `pip install -r sensr_lidar_recorder/requirements.txt` aligns local packages with the recorder stack.
- Ubuntu 환경에서는 `pip install -r sensr_lidar_recorder/requirements_ubuntu.txt`를 사용하여 의존성을 설치하세요.
- `python sensr_lidar_recorder/main.py --config config/config.yaml --host <id>` runs the recorder; use `--list-hosts` to discover IDs and `--pointcloud-only` for lightweight runs.
- `bash ros_test/build_and_run.sh` or `docker compose -f ros_test/docker-compose.yml up` builds and launches the ROS 2 bridge with rviz and bag recording enabled.

## Coding Style & Naming Conventions
- Python modules follow 4-space indentation, type hinted function signatures, and docstrings in English even when inputs originate from Korean data sources.
- Keep filenames snake_case and class names PascalCase; mirror existing patterns like `sensr_manager.py` and `ObjectList.msg`.
- Run `black sensr_lidar_recorder/src` and `flake8 sensr_lidar_recorder/src` before opening a review; update `requirements.txt` comments if new tools become mandatory.
- ROS message fields stay lower_snake_case and should document units (`velocity_mps`, `timestamp_ns`).

## Testing Guidelines
- Place unit tests under `sensr_lidar_recorder/tests/` and execute them with `pytest -q sensr_lidar_recorder/tests`; ensure mock ROS clients cover Windows fallback paths.
- Use `ros_test/test_system.sh` for end-to-end validation against a live SENSR endpoint; capture bag excerpts in `ros_test/src/sensr_ros2_driver/scripts/` when debugging.
- Target 80% statement coverage on new Python modules and include regression cases for protobuf schema changes.

## Commit & Pull Request Guidelines
- The distributed snapshot lacks VCS metadata; adopt short, imperative commit subjects (e.g., `Add bag recorder reconnection guard`) followed by body context when needed.
- Reference Jira or GitHub issues in the footer using `Refs:` and attach comparison metrics (throughput, CPU) whenever recorder performance shifts.
- Pull requests should list affected scripts, required config changes, and screenshots of rviz topics when UI layers change. Request a ROS reviewer for any `.msg` or launch edits.

## Security & Configuration Tips
- Do not hard-code production hosts (e.g., `112.133.37.122`) in checked-in configs; keep overrides in `.env.local` or untracked YAML copies.
- Scrub sample bags from `output/` before pushing; large files and PII radar snapshots belong in the shared staging bucket only.
- Rotate API tokens referenced in `CONFIGURATION.md` per release and store new credentials in the deployment secrets manager, not in this repo.
