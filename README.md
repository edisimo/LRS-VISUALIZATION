# LRS Flight Lab

An offline, interactive 3D lecture companion for Master's students in Cybernetics and Robotics. Four workspaces explain map representations, path planning, trajectory processing and local avoidance in the FEI hangar.

## Run

Python 3.10+ and a desktop WebGL2 browser are sufficient. No ROS, Gazebo, npm installation, internet access or preprocessing is needed for playback. Linux/Bash is the primary launcher environment; the Python server also works on Windows/macOS.

```bash
cd LRS-VISUALIZATIONS
./run_mapping.sh
./run_planning.sh
./run_trajectory.sh
./run_avoidance.sh
```

Run **one** launcher, then switch topics in the same browser tab. It opens `http://127.0.0.1:8765`. If that port is already in use, use the existing tab, or pass `--port 8766`. For manual browser opening, add `--no-browser`.

```bash
python3 scripts/serve.py --topic mapping --no-browser
```

All JavaScript and datasets are local. The server binds to loopback. Opening `index.html` directly is unsupported because modules and dataset loading need HTTP. The former Complete pipeline section and its launcher have been removed.

## Mapping: separate geometry, storage and information

The selector is ordered in three groups:

1. **Geometry/storage:** point cloud, dense voxels, sparse voxels, blocks, octree, triangle mesh and elevation map.
2. **Collision processing:** inflation around obstacles only.
3. **Information in space:** occupancy, cost and ESDF, kept together with the same slice/probe.

### Geometry and storage

- **Point cloud:** 53,799 original surface points, downsampled from the 620,311-point PCD. Point size and display sampling are adjustable. Missing points do not imply observed free space.
- **Dense voxels:** allocate all `N = Nx Ny Nz` cells in the bounded 10 × 10 × 5 m volume. Floor markers show one allocation slice; occupied surface samples are cubes.
- **Sparse voxels:** the identical occupied surface cells, with only `K` entries allocated in a hash map. The current representation's memory estimate is prominent; both estimates and their ratio are shown for comparison.
- **Voxel blocks:** a sparse directory of 2 m blocks containing dense local grids. Playback moves a local region of interest.
- **Octree:** a 10 m root cube at `(0,2,0)`, with genuine eight-child subdivisions near samples and coarse empty leaves. Maximum depths **1–6** are selectable. Geometry is batched so deeper trees do not require one draw call per leaf.
- **Triangle mesh:** 48 triangles describe the surfaces of the same four simplified rack envelopes. This is analytic model geometry, not a reconstructed mesh from the cloud. Connectivity stores surfaces rather than all volume cells.
- **Elevation map:** one highest observed height `h(x,y)` per XY column, derived from the PCD. This compact **2.5D** representation loses stacked surfaces and space below roofs/shelves, illustrating its limitations for indoor UAVs.

Voxel/block/elevation cell widths are **0.10, 0.25 and 0.50 m**, defaulting to **0.10 m**. A display cell smaller than the cloud's 0.16 m downsampling does not create additional measured detail.

**Why sparse is not always smaller:** dense occupancy is estimated at `N × 1 byte`; a hash entry at `K × 24 bytes` (1 byte value plus 23 bytes of assumed coordinates/indexing overhead). Sparse wins when `K/N < 1/24`, under these assumptions. A coarse, highly occupied grid can make the hash map larger. These estimates represent alternative implementations of the same values, not browser heap measurements. Octree storage uses an illustrative 40 bytes per node; block payloads exclude directory overhead; elevations use float32 heights and exclude the XY index.

### Inflation

Only the rack obstacles are inflated: **no outer building/crop boundary and no paths**. A drone marker with its physical radius and safety shell supplies scale.

In grid mode a horizontal slice shows the occupied cells and the additional forbidden cells. The display computes `n = ceil((robot radius + margin) / cell width)` and uses a conservative rounded distance threshold `n × cell width` around analytic rack surfaces. This is a quantized Euclidean-offset illustration, rather than a Chebyshev cube dilation of a raster map. Playback grows this threshold from zero. Controls change cell width, radius, margin and slice altitude.

In octree mode, adaptive refinement occurs near the offset surface. Leaves are conservatively marked when `distance(center) − half-diagonal ≤ radius + margin`. This demonstrates mixed-size leaves intersecting the inflated region; “number of extra cells” is not a single meaningful value for an adaptive tree. Depth controls the finest available width.

### Occupancy, cost and ESDF

These are **different values associated with the same location**, not competing storage structures. All three use the same 0.25 m horizontal sample slice of the rack-envelope model, retain the selected slice/probe when switching, and omit routes and start/goal markers.

| View | Question | Value / visualization |
|---|---|---|
| Occupancy | Is there an obstacle **here**? | 0 = free, 1 = occupied; binary green/coral colors |
| Costmap | How undesirable is this location? | `100 exp(−max(d,0)/falloff)`; arbitrary penalty 0–100 with a labeled color ramp |
| ESDF | How far is the nearest surface? | Signed metres; click a point to see a white nearest-surface ruler |

These views measure distance to **racks only**. The model supplies known free space; unknown occupancy and sensor ray integration are not simulated. Inflation is a separate collision-processing view, so a cost penalty is not silently confused with binary occupancy.

## Planning

Dijkstra, A*, weighted A* and Theta* all use the **same 26-neighbour, 0.5 m 3D graph**, Euclidean edge weights, and 0.35 m collision radius. Every diagonal edge is swept-segment collision-checked; no corner cutting is allowed. Theta* additionally tests visible ancestors. Its standalone view no longer overlays A*.

RRT, RRT* and Informed RRT* replay seeded samples, parent selection and actual rewiring. The informed ellipsoid appears only after a first solution. These deliberately small runs use a fixed neighbour radius and are not asymptotic performance benchmarks.

**PRM** first samples free configurations, connects nearby visible samples, and then searches the resulting reusable roadmap. Playback separates sampling, edge construction and graph search. The prepared example uses 350 vertices and 14 nearest-neighbour candidates per vertex.

**D* Lite** retains `g`, one-step lookahead `rhs`, and its priority queue from the initial backward search. The UAV advances one original grid edge; the moving-start key offset `km` changes. A new obstacle raises affected edge costs, inconsistent vertices are updated, and the saved search is repaired. Orange marks changed-edge endpoints and yellow marks actual repair expansions. This is a real incremental implementation, not a fresh A* search relabelled as D* Lite.

**Comparison** has independent checkboxes for every planner, all enabled by default. It compares initial-map routes, including D* Lite's initial route; the repaired route is deliberately kept out of that unchanged-map comparison. Measured preparation runtime, expansion/vertex count, path length and minimum clearance update with visibility. Counts across graph and sampling planners represent different quantities; timings describe these implementations in this scene only.

All planners conservatively expand rack AABBs by 0.35 m and contract planning bounds. Mapping inflation intentionally omits the bounds for visual clarity. AABB inflation is more restrictive at corners than a Euclidean spherical offset.

## Paths and trajectories

The processing examples deliberately retain the **6-neighbour stepped A* baseline** to make waypoint removal legible. This is labelled in the UI; it does not change the 26-neighbour planner comparison.

- **Raw A*** displays every waypoint.
- **Line-of-sight pruning** tests farthest-visible waypoint connections; accepted tests are green, rejected tests red.
- **Random shortcutting** tries seeded waypoint pairs and accepts validated shorter connections.
- **Spline smoothing** uses a cubic B-spline approximation that actually cuts into obstacles; unsafe samples are red.
- **Minimum snap / minimum jerk** solve joint polynomial smoothness QPs with fixed segment times, waypoint constraints and derivative continuity. Seventh-degree minimum snap is continuous through jerk; fifth-degree minimum jerk through acceleration. These solves do not impose collision, thrust or actuator constraints; sampled clearance is reported.
- **CHOMP-inspired optimization** uses covariant smoothness/obstacle gradient descent and a feasibility-preserving line search. Orange directions illustrate the distance gradient; objective history is shown.
- **STOMP-inspired optimization** samples 24 correlated perturbations per iteration, computes exponential cost weights and proposes a weighted update. Eight trial trajectories are drawn. Only feasible objective improvements are accepted; the implementation may also choose an improving sampled trial. This simplified whole-trajectory scoring variant omits full STOMP per-timestep cost-to-go weighting. It uses no obstacle-cost gradient.

The former Path vs trajectory vignette has been removed. Equations and concise algorithm logic appear directly in each remaining view.

## Local avoidance

All methods now use the **same rack scene, start, goal and new suspended obstacle**. Potential fields no longer use a specially constructed U-shaped trap. The shared global reference is drawn purple where appropriate, local decisions green, and executed motion white.

| Method | Decision rule | Teaching limitation |
|---|---|---|
| A* replanning | Fresh search after the map changes | Global replanning, not a reactive controller |
| D* Lite replanning | Repair retained `g/rhs` consistency | Same incremental sequence as in Planning |
| Potential field | Goal attraction plus nearest-surface repulsion | Local minima can occur even in the actual racks; no forced-success fallback |
| Bug2 | Seek goal, follow a boundary, leave at a closer m-line crossing | Finite-step **2D, fixed-altitude adaptation**; not a general 3D flight planner |
| DWA | Sample acceleration-reachable velocities, reject unsafe stopping rollouts, score progress | Holonomic **3D adaptation** of a method originally developed for ground robots; local minima remain possible |
| VFH | Build a binary angular obstacle histogram and choose a free sector | **2D binary teaching variant**, not full VFH/VFH+; cannot exploit vertical escape |
| MPC | Optimize a bounded finite-horizon velocity sequence; execute its first control | Simplified single-integrator dynamics with a global-detour terminal guide |
| MPPI | Exponentially weight 64 noisy control sequences; execute the weighted first command | Simplified weighted shooting, without the full path-integral noise/control correction |

Reactive methods display their real outcomes, including failure to reach the goal. The 3D collision filter prevents executing invalid segments; it does not secretly replace a failing method with A*. MPC/MPPI use detour guidance, while the simple reactive examples seek the goal directly, so arrival success is not an apples-to-apples controller benchmark.

MPC/MPPI use a separate, validated 6-neighbour detour guide (independent of the 26-neighbour planner comparison) and retain their prepared 6/12/18-step horizons and 0.22 s action period. The rollout-count slider changes how many of the 24 saved alternatives are drawn, not the 64 samples used by MPPI. MPC's perturbation lines illustrate candidate alternatives rather than optimizer iterations.

Velocity obstacles, ORCA and CBF are possible extensions, not included in this revision. ORCA would benefit from an explicit reciprocal multi-agent scenario; moving-obstacle prediction is outside the current stationary-after-detection example.

## Controls

Left drag orbits, wheel zooms, right drag pans. Perspective/Top/Camera buttons reset the view. Space plays/pauses; Right arrow steps; R resets parameters, progress and camera; C resets the camera; **1–4** select topics. The timeline scrubs prepared histories, and speed selects 0.5–4× playback. Shortcuts do not intercept focused form controls. The fullscreen button is available in the header.

Click any of Occupancy/Costmap/ESDF to select the shared probe. Comparison checkboxes show/hide routes and their metric rows. Reset restores all comparison routes. A desktop display of at least 1280 × 800 is recommended.

## Precomputation and tests

Normal launch reads bundled binary/JSON assets. Mapping and information views compute locally in JavaScript. Algorithm histories, including D* Lite repairs, PRM, STOMP and reactive-control traces, are prepared in Python.

```bash
# Only for regenerating data:
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
./scripts/precompute_all.sh
# Optional explicit source repository:
./scripts/precompute_all.sh /path/to/LRS-URK

# Asset, cache, collision, launch, math and syntax checks:
./scripts/test_all.sh

# Optional Chromium development smoke checks (server on port 8765):
npm ci
npx playwright install chromium
./run_mapping.sh --no-browser
# In another terminal:
node tests/browser.mjs
node tests/revision.mjs
```

`precompute_all.sh` prefers `.venv/bin/python`. Preparation is an explicit offline operation and can take minutes; never run it during a lecture. Seeds are fixed; measured timings change between regenerations. `scripts/refresh_revision.py` updates algorithm histories against existing source assets/controller caches and is also called by complete preparation.

## Assets and architecture

`LRS-URK` remains unchanged. `maps/FEI_LRS_PCD/map.pcd` supplies the downsampled XYZ asset; `assets/provenance.json` records its hash, counts and original bounds. Four 3.09 × 1.05 m shelf footprints come from `models/fei_lrs_racks/model.sdf`, represented by conservative solid 3.2 m envelopes. Hangar SDF/DAE/world files were inspected for context. The large drone meshes were inspected but not copied; a procedural UAV marker keeps the views legible. Line-of-sight pruning follows the assignment's stated concept.

`common/scene.js` supplies rendering/camera/colors; `common/math.js` supplies browser spatial operations; `common/algorithms.py` and `common/extensions.py` compute actual algorithm histories. Each topic owns a `view.js`; `app.js` owns shared controls/playback. `precomputed/lecture.json` stores histories, and `vendor/` contains offline Three.js r180 and its MIT notice. Source-derived data retains its original ownership and source terms.

Algorithm references: [D* Lite, Koenig and Likhachev](https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf), [Dynamic Window Approach, Fox, Burgard and Thrun](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance). Implementations here are deliberately small teaching examples; their specific adaptations are described above.
