# hiroz_nav

Work-in-progress navigation stack built on top of `hiroz`.

Status
- **WIP:** This repository is a work in progress. All feedback, issues, and guidance are appreciated - please open issues or send PRs if you have suggestions for architecture, ROS/Zenoh integration, or testing.

Overview
- This crate workspace contains ROS-bridge and navigation components used during development at my local workspace: planner, controller, mapio, costmap and a `gazebo_bridge`.
- The crates depend on the main `hiroz` crates via workspace-relative path dependencies during development. See "Development" below for how to build locally.

License
- This project follows the same license as the parent `hiroz` repo (Apache-2.0). See the top-level `LICENSE` in the original workspace.

Development
- Recommended: clone the parent `hiroz` workspace so the path dependencies resolve. Example layout:

  parent-folder/
  ├─ hiroz/        # the main project used by the crates via path deps
  └─ hiroz_nav/    # this repo

- From `parent-folder` you can build the workspace:

```bash
cargo build --workspace
```

- If you only want to work on `hiroz_nav` alone, you must either:
  - Replace the `path = "../../hiroz/..."` dependencies in each crate's `Cargo.toml` with published crate versions, or
  - Add the missing `hiroz` crates as path dependencies inside this repository (not recommended unless you intend to vendor them).

Run example (local)
--
The following is an example of how I run the navigation stack locally using multiple terminals. Adjust paths and environment variables to match your setup.

Terminal 1:

```bash
export TURTLEBOT3_MODEL=burger
zenohd &
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 rust_ros2/rust_nav_stack/rviz_bridge.py
```

Terminal 3:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
rviz2 -d ~/tb3_hiroz.rviz
```

Terminal 4:

```bash
~/.cargo-target-shared/release/planner --mode client --endpoint tcp/127.0.0.1:7447
```

Terminal 5:

```bash
~/.cargo-target-shared/release/controller --mode client --endpoint tcp/127.0.0.1:7447
```

Notes:
- The shared cargo target directory (your local setup) may differ; some setups use `~/.cargo-shared-target` or other paths - adjust the paths above accordingly.
- If you use a shared `CARGO_TARGET_DIR`, export it before building so the `planner`/`controller` binaries appear in the expected release folder.

CI / GitHub Actions
- Because these crates commonly use workspace-relative path dependencies to `hiroz`, the included GitHub Actions workflow currently only checks formatting (`cargo fmt -- --check`). If you want CI to run `cargo build`/`cargo test`, we can either:
  - Vendor the `hiroz` crates into this repo, or
  - Publish the dependent crates to crates.io and update the `Cargo.toml` entries, or
  - Configure CI to fetch the upstream `hiroz` repository into the runner before building.

Files of interest
- `planner/`, `controller/`, `mapio/`, `costmap/`, `gazebo_bridge/` - the workspace crates.

Please tell me if you want me to:
- create the GitHub repo and push these files, or
- just prepare the local commits and give you exact push steps (recommended if you prefer to control repo creation).

Thank you - any guidance or review is welcome!
