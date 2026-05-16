# test_traj_panel

ROS 2 **RViz2 panel plugin** that publishes repeatable sinusoidal test motions as **`geometry_msgs/PoseStamped`**, a TF **child frame**, and a **`nav_msgs/Path`** trace—useful for exercising planners, controllers, or visualization pipelines without a simulator.

## Requirements

- ROS 2 (developed against **Rolling**; should work on other distros with compatible `rviz_common`)
- RViz2
- Build deps: `ament_cmake`, `qtbase5-dev` (see `package.xml`)

## Build

From your ROS 2 workspace:

```bash
cd /path/to/your_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select test_traj_panel
source install/setup.bash
```

## Load the panel in RViz2

1. Start RViz2 (`rviz2`).
2. **Panels → Add New Panel → test_traj_panel → TestTrajPanel** (class id `test_traj_panel/TestTrajPanel`).
3. Configure topics, frames, oscillation parameters, then press **Start**. Press **Stop** to halt publishing.

The plugin uses RViz’s shared ROS node and clock (`use_sim_time` follows RViz when enabled).

## Published outputs

| Output | Type | Default topic / frame |
|--------|------|------------------------|
| Target pose | `geometry_msgs/msg/PoseStamped` | Topic: `/curent_target` |
| Path trace | `nav_msgs/msg/Path` | Topic: `/test_traj_panel/path` |
| Target frame | TF (`tf2_ros`) | Child: `test_traj_target`; parent: RViz **Fixed Frame** (unless overridden) |

- **Parent frame**: Leave empty to use the RViz **Global Options → Fixed Frame**; set explicitly to force a different parent.
- **Publish rate**: Timer-driven publishing frequency (Hz); default **30 Hz**.
- **Start** clears the internal path buffer and resets motion time from zero.

The path message accumulates poses while running (last **5000** poses kept).

## Motion model

For each axis *i* ∈ {X, Y, Z}, position is offset from the configurable **zero pose** by:

\[
\Delta_i(t) = A_i \sin(2\pi f_i t + \phi_i)
\]

- \(A_i\): magnitude (meters); negative magnitude flips phase.
- \(f_i\): frequency (Hz); **\(f_i \le 0\)** disables oscillation on that axis (offset is zero).

Phase offsets (fixed):

| Axis | \(\phi_i\) |
|------|------------|
| X | \(0\) |
| Y | \(\pi/2\) |
| Z | \(0\) |

So **equal magnitude and frequency on X and Y** traces a **circle** in the XY plane. **Different frequency ratios** produce **Lissajous** curves in XY (and similar behavior when Z is excited). Z shares the same phase reference as X for XZ / 3D combinations.

Orientation is constant: quaternion from the panel **roll / pitch / yaw** (radians) applied at the zero pose (defaults: zero translation, identity rotation).

## Visualizing in RViz

Recommended displays:

1. **TF** — Shows the moving **target** frame relative to the parent.
2. **Path** — Subscribe to your **Path topic** to see the trajectory trace.
3. **Pose** (optional) — Subscribe to your **Pose topic** for an arrow at the current target.

Ensure **Fixed Frame** matches how you think about the parent (or set **Parent frame** in the panel).

## Building with Qt5 vs Qt6

`CMakeLists.txt` disables discovery of Qt6 (`CMAKE_DISABLE_FIND_PACKAGE_Qt6`) and links **Qt5**, which matches many Ubuntu + RViz Rolling setups and avoids mixed Qt major-version errors.

If your RViz is built **Qt6-only**, you may need to remove that guard and align Qt discovery with your platform so the plugin and RViz use the **same** Qt major version.

## License

Apache-2.0 (see `package.xml`).
