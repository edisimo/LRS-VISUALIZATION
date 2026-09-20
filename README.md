# LRS Flight Lab

An offline, interactive 3D lecture companion for Master's students in Cybernetics and Robotics. Five connected workspaces explain how surface samples become a map, a route, a timed trajectory, and finally obstacle-aware UAV motion.

The real FEI hangar point cloud establishes the setting. A small, conservative model of the same shelving units keeps the algorithm demonstrations readable. All scenes use metres and Z-up coordinates; the planning start is `(4, 3, 1.5)` and goal is `(5, 9, 1.5)`.

## Quick start

**No package installation, ROS, Gazebo, or preprocessing is needed for lecture playback.** Use Python 3.10+ and a modern desktop browser with WebGL2. Linux is the primary target; the Python launcher also works on macOS/Windows. Shell launchers require Bash.

```bash
cd LRS-VISUALIZATIONS

./run_mapping.sh
./run_planning.sh
./run_trajectory.sh
./run_avoidance.sh
./run_full_demo.sh
```

Run **one** of these commands. Each starts the same application at the requested tab and opens `http://127.0.0.1:8765`. Switch among all five tabs without restarting. Press **Play** to animate; the complete sequence lasts 84 seconds at 1×. Stop the server with Ctrl+C.

If a server is already running, use its tabs or stop it before starting another launcher. An additional instance can use `--port 8766`. On a remote machine or when you want to open the browser yourself:

```bash
./run_full_demo.sh --no-browser --port 8765
# Cross-platform equivalent:
python3 scripts/serve.py --topic full_pipeline --no-browser
```

The launcher binds only to loopback. The browser loads locally vendored Three.js and prepared data, with no CDN, network account, fonts, API keys, or runtime Python packages. Opening `index.html` directly as a file is not supported because browser modules and data fetches require a local HTTP server.

## What to show

### 1. Map representations

| Demo | What students see | Controls |
|---|---|---|
| Point cloud | 53,799 original measured surface samples, downsampled from 620,311 | Point size; display subsampling |
| Dense voxels | Occupied cubes plus markers for all allocated cell centers | 0.25 / 0.5 / 1 m cell size |
| Sparse voxels | Identical occupied cells, allocated individually | Same resolutions; memory comparison |
| Voxel blocks | A rolling region allocates 2 m blocks with dense cells inside | Cell size; playback/scrub |
| Octree | Every split produces eight children; empty regions remain coarse | Maximum depth 1–4 |
| Occupancy | Surface samples overlaid with model-derived solid occupancy | Slice altitude; reveal playback |
| Inflation | Drone radius and margin expand forbidden space; the aisle closes | Radius; margin; playback |
| Costmap | Exponential clearance penalty, plus short and clearance-optimized paths | Slice altitude; falloff length |
| ESDF | Signed Euclidean distance to solid boxes and crop boundaries | Slice altitude; color range; click to probe |

Dense/sparse voxelization uses the **real downsampled PCD**, cropped to `[0,10] × [2,12] × [0,5]` m. The complete cloud remains visible in the raw-cloud scene. The octree uses a 10 m root cube at `(0,2,0)` so it has equal side lengths.

Occupancy, inflation, costs, distance queries and planning use **solid rack envelopes**, not a sensor-derived free-space reconstruction. Missing PCD samples are never treated as evidence of observed free space. The envelope model conservatively fills shelving gaps and assumes the rest of the bounded scene is known free; unknown occupancy is not simulated. The 0.5 m occupancy cubes are center samples, not exact surface rasterization.

Memory numbers are explanatory payload estimates: dense occupancy = 1 byte per cell; sparse entry = 24 bytes including assumed key/hash overhead; octree = 40 bytes per node; blocks = 1 byte per internal cell excluding block-table overhead. They are **not browser heap measurements**. Sparse storage can use more memory than a compact dense array at high occupancy.

### 2. Path planning

Dijkstra, A*, weighted A* (`w=2.5`) and Theta* replay actual searches on the same 0.5 m, 6-connected **3D** graph. Blue points show expanded nodes; yellow shows actual queued frontier snapshots, recorded every 20 expansions. Theta* performs ancestor line-of-sight checks during search; the purple A* baseline makes the any-angle change visible.

RRT, RRT* and Informed RRT* replay seeded continuous 3D samples, parent selection, and tree edges. RRT* updates descendant costs after rewiring; pink highlights rewires from the latest eight accepted samples. The best-path history contains only genuine improvements. Informed RRT* samples a 3D prolate hyperspheroid after finding a route. The finite teaching runs use 1,100 attempts, 0.85 m extension and 1.65 m neighbor radius; this fixed-radius illustration is not an asymptotic-performance benchmark.

**Comparison** overlays Dijkstra, A*, Theta*, RRT, RRT* and Informed RRT* and lists preparation time, vertices/expansions, length and sampled minimum clearance. Graph expansion counts and sampling-tree sizes are different quantities. Timings are actual wall-clock measurements of the Python preparation runs on the machine that generated the cache, not playback timings or universal algorithm rankings.

All planners use an exact segment/AABB slab test with each rack box expanded by 0.35 m and the outer bounds contracted by 0.35 m. This is a conservative swept-sphere check: box corners are more restrictive than Euclidean-radius inflation. Mapping's Euclidean inflation visualization therefore differs slightly near corners. The mapping radius controls do **not** modify prepared planner histories; the app labels this explicitly.

### 3. Paths and trajectories

Raw A* exposes each grid waypoint. Line-of-sight pruning tries the farthest reachable waypoint, displaying accepted and rejected edges. Seeded random shortcutting removes intermediate points only after a collision check. Both retain the finite-robot collision model.

Cubic B-spline approximation of the pruned path demonstrates collision risk: unsafe samples are red. The path-versus-trajectory vignette uses a sharp 90° corner, a moving UAV, position/velocity/acceleration readouts and a speed chart. Its two four-second, seventh-degree segments stop at the corner; attempting the turn at nonzero speed instead requires a discontinuous velocity.

Minimum snap solves a **joint quadratic program** for seventh-degree piecewise polynomials, minimizing integrated squared fourth derivative, with fixed waypoint positions, zero endpoint derivatives and continuity through jerk. Minimum jerk similarly uses fifth-degree polynomials and continuity through acceleration. Segment durations are fixed from distance, not optimized. These are genuine smoothness minimizers, but **do not impose collision, thrust, attitude or actuator constraints**. The view reports sampled clearance and colors violations red; a smooth result is not presented as automatically flyable.

CHOMP is explicitly **CHOMP-inspired**: covariant descent uses a second-difference smoothness metric and analytic-box signed-distance penalties, with finite-difference gradients and collision-checked backtracking. It illustrates smoothness/obstacle forces and objective reduction, but omits CHOMP's full arc-length-weighted functional and continuous robot dynamics. The orange line segments show distance ascent directions; the green curve and objective chart follow saved iterations.

### 4. Local obstacle avoidance

A* replanning introduces a suspended load intersecting the original route. The UAV stops at a detection point, the collision map changes, a fresh A* search finds a detour, and motion resumes. This is ordinary replanning, **not D* Lite**.

The potential-field vignette integrates attractive and nearest-surface repulsive forces in a separate, symmetric U-shaped obstacle layout. Opposing forces trap the UAV in a local minimum. Cyan, coral and yellow vectors show attraction, repulsion and the sum.

MPC solves a bounded, single-integrator 3D velocity-control problem with SciPy L-BFGS-B. MPPI is a simplified sampling controller: 64 temporally correlated velocity perturbations are scored, exponentially weighted, and combined into a command. It omits the full stochastic path-integral change-of-measure/control-noise correction; it teaches weighted shooting rather than claiming to reproduce a full flight controller.

Both execute one 0.22 s action and shift the horizon. Choose **6, 12 or 18 prediction steps** to load distinct computed runs immediately. Display 4–24 candidate rollouts without changing the 64 samples used by MPPI. MPC's displayed perturbations illustrate alternatives; they are not L-BFGS-B iterates. Purple = original global path; green = predicted local motion; white = executed motion; yellow = first action; coral = obstacle/violating rollout.

A collision-free A* detour provides successive terminal guide waypoints to escape local minima. Consequently this is **detour-guided predictive tracking**, not proof that a local controller discovers a way around arbitrary obstacles. A first-action collision filter can stop an invalid weighted control; its status is displayed. The suspended load is initially unknown but stationary after detection. Moving-obstacle prediction and full quadrotor dynamics are not implemented.

### 5. Complete pipeline

Twelve highlighted stages connect hangar geometry, surface cloud, voxel occupancy, inflation, A* search, route extraction, pruning, timed motion, a newly detected suspended load, replanning, repaired execution and arrival.

This is an educational composition: surface voxelization transitions to model-derived conservative rack occupancy. Timed execution uses seventh-degree **stop-to-stop minimum-snap interpolation on validated straight segments**, rather than executing the unconstrained joint polynomial example. The UAV stays at its detection position during replanning, and the repaired trajectory starts at that same position. It stops at each remaining corner; no dynamic-feasibility claim is made.

## Presentation controls

| Input | Action |
|---|---|
| Left drag | Orbit |
| Wheel / trackpad | Zoom |
| Right drag | Pan |
| Perspective / Top / Camera | Restore a preset view |
| Play / Space | Play or pause (restarts if at end) |
| Step / Right arrow | Advance one search/controller event, or a small step in other views |
| Timeline | Scrub deterministically |
| Speed | 0.5×, 1×, 2×, 4× playback |
| Reset / R | Restore parameter defaults, progress and camera |
| C | Reset camera |
| 1–5 | Select a topic |
| Fullscreen icon | Browser fullscreen |
| Click ESDF slice | Inspect distance at that location |

Keyboard shortcuts are disabled while a control has focus so they do not interfere with normal input. Use a desktop screen, preferably 1280 × 800 or larger. See [LECTURE_ORDER.md](LECTURE_ORDER.md) for suggested sequences.

## Regenerating prepared data

Normal launch never preprocesses. Distributed assets include a 0.65 MB little-endian float32 XYZ cloud and a few MB of JSON histories. Browser voxelization, octree construction, distance slices, inflation and the small potential-field example compute locally. Planning, pruning, polynomial solves, CHOMP iterations and controller rollouts are cached.

For regeneration only, install NumPy and SciPy in an isolated environment. This avoids conflicts with ROS/system Python packages:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
./scripts/precompute_all.sh
# If LRS-URK is not the sibling directory:
./scripts/precompute_all.sh /path/to/LRS-URK
```

The script prefers `.venv/bin/python` when present. Preparation takes substantially longer than playback; do it before the lecture. Fixed seeds make geometry and algorithm histories reproducible within numerical tolerances; measured runtimes naturally change. This environment was verified with NumPy 2.2.6 and SciPy 1.15.3.

## Source assets and relation to LRS-URK

`LRS-URK` is left unchanged. The application works without it after preparation.

- `maps/FEI_LRS_PCD/map.pcd`: original 620,311-point ASCII PCD. One original point is retained per 0.16 m voxel; the XYZ subset is stored in `assets/hangar-points.bin`. No synthetic points are substituted. `assets/provenance.json` records source hash, bounds and counts.
- `models/fei_lrs_racks/model.sdf`: four shelf locations and 3.09 × 1.05 m footprints. Solid 3.2 m tall envelopes are an explicitly conservative educational simplification.
- `models/fei_lrs_hangar/model.sdf`, `models/hangar/*.dae`, and `worlds/fei_lrs_gazebo.world`: inspected for coordinate/world context. Large textured meshes are not copied.
- `models/fei_lrs_drone/drone_lowres.dae` and `propeller.dae`: inspected; the app uses a small procedural quadrotor marker for consistent readability, rather than copying 6.9 MB of meshes.
- `assignments/assignment1/01_map_and_path_planning.md`: the farthest-visible pruning demonstration follows the assignment's line-of-sight shortcutting concept. No pre-existing planner implementation was found in this repository.

Source-derived data retains its original ownership and applicable source terms. Three.js r180 is vendored with its MIT notice in `vendor/THREE-LICENSE`.

## Architecture

```text
app.js / index.html / style.css   Shared UI, playback and layout
common/scene.js                   Three.js camera, geometry and shared colors
common/math.js                    Browser voxelization, octree, signed distances
common/catalog.js                Concise teaching copy and topic definitions
common/algorithms.py              Search, collision checks, minimum derivatives, optimizer
mapping/view.js                  Mapping and storage scenes
planning/view.js                 Search-history and comparison scenes
trajectory/view.js               Path processing and trajectory scenes
avoidance/view.js                Replanning, potential field, predictive-control scenes
full_pipeline/view.js            Integrated twelve-stage sequence
assets/                          Small source-derived cloud and provenance
precomputed/lecture.json          Actual algorithm histories and recorded metrics
vendor/                          Offline Three.js modules and license
scripts/                         Launcher, preparation and checks
tests/                           Cache/collision, launch, math and browser smoke tests
```

Instanced cubes keep voxel views compact, and playback rebuilds scene geometry at a capped update frequency while camera rendering remains independent. Prepared JSON and the binary cloud load concurrently. There is no application backend beyond Python's local static file server.

Three.js rendering uses [InstancedMesh](https://threejs.org/docs/pages/InstancedMesh.html) and [OrbitControls](https://threejs.org/docs/pages/OrbitControls.html). For the mathematical context of polynomial smoothness optimization, see the primary research paper [Generating Minimum-Snap Quadrotor Trajectories Really Fast](https://arxiv.org/abs/2008.00595); the small dense QP here is a teaching implementation, not that paper's optimized solver.

## Verification

```bash
./scripts/test_all.sh
```

The standard-library Python tests verify asset counts, prepared data, feasible planner paths, genuine rewiring and cost improvements, polynomial constraints, controller goal arrival, collision-free executed segments, and all five launchers from a different working directory. If Node is available, the script also runs math tests and syntax-checks all application modules.

For an actual Chromium/WebGL smoke test (development dependencies only):

```bash
npm ci
npx playwright install chromium
./run_full_demo.sh --no-browser
# In another terminal:
node tests/browser.mjs
```

The browser check visits every concept, checks the DOM and browser errors, and captures representative screenshots. It requires the server on port 8765. Linux environments missing Chromium system libraries may need `npx playwright install-deps chromium`.

## Deliberate scope

The 20 priority concepts are covered; secondary additions include weighted A*, minimum jerk, random shortcutting, voxel blocks and the potential-field failure. D* Lite, RRT-Connect, PRM/PRM*, BIT*, STOMP, TrajOpt, velocity obstacles, ORCA and TSDF are not included. Useful next extensions would be moving-obstacle predictions, sensor raycasting for unknown/free occupancy, constrained joint minimum-snap optimization, and interactive live planning beyond the prepared examples.
