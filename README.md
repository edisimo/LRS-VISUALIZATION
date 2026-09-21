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

Voxel/block/elevation cell widths are **0.10, 0.25 and 0.50 m**, defaulting to **0.25 m**. A display cell smaller than the cloud's 0.16 m downsampling does not create additional measured detail.

**Why sparse is not always smaller:** dense occupancy is estimated at `N × 1 byte`; a hash entry at `K × 24 bytes` (1 byte value plus 23 bytes of assumed coordinates/indexing overhead). Sparse wins when `K/N < 1/24`, under these assumptions. A coarse, highly occupied grid can make the hash map larger. These estimates represent alternative implementations of the same values, not browser heap measurements. Octree storage uses an illustrative 40 bytes per node; block payloads exclude directory overhead; elevations use float32 heights and exclude the XY index.

### Inflation

Only the rack obstacles are inflated: **no outer building/crop boundary and no paths**. A drone marker with its physical radius and safety shell supplies scale.

**Whole volume** is the default in both voxel and octree modes. The orange voxel exterior shell surrounds the racks on all sides within the teaching volume; its entire enclosed region is forbidden, including hidden interior cells. Choose **Horizontal slice** to inspect individual layers; the altitude slider appears only for that view. Octree slices retain only leaves intersecting the selected altitude.

In grid mode the display computes `n = ceil((robot radius + margin) / cell width)` and uses a conservative rounded distance threshold `n × cell width` around analytic rack surfaces. This is a quantized Euclidean-offset illustration, rather than a Chebyshev cube dilation of a raster map. Playback grows this threshold from zero. Controls change cell width, radius, margin, view type and (in slice mode) altitude.

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

**D* Lite** in Path planning shows the initial backward search on the unchanged map and reveals the route when the search finishes. In Local avoidance, **D* Lite replanning** starts with that route already planned and animates flight, obstacle detection, a pause for repair, and execution to the goal. It retains `g`, one-step lookahead `rhs`, and its priority queue from the initial backward search. The UAV advances one original grid edge; the moving-start key offset `km` changes. A new obstacle raises affected edge costs, inconsistent vertices are updated, and the saved search is repaired. Orange marks changed-edge endpoints and yellow marks actual repair expansions. This is a real incremental implementation, not a fresh A* search relabelled as D* Lite.

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

## Local avoidance: working 3D adaptations

All four reactive examples now **reach the goal, avoid collisions and change altitude** in the prepared rack scene. Their histories are real computations, not interpolations of a replacement planner's route. This validates the displayed example, not arbitrary environments or physical flight dynamics.

Classic Bug1/Bug2 and VFH were developed for planar navigation. There are published 3D Bug variants, including 3DBug and Frustumbug, and 3DVFH/3DVFH+ methods for UAVs. The lightweight implementations here preserve the central ideas with analytic obstacle geometry rather than reproducing a stereo-vision or point-cloud sensor stack.

| Method | What was improved | Visible internals |
|---|---|---|
| **Bug 3D** | 3DBug-inspired sampling of inflated obstacle faces/edges in XYZ. Follow nearby ordered samples of the original global route. When blocked, choose a visible local boundary target, remember visited samples, and rejoin the first clear route sample beyond the obstruction. | Candidate boundary connections, purple route target, yellow active target, track/detour/rejoin state and altitude |
| **Potential field** | Track the original route with attraction and repulsion, plus a tangential escape term. Surface normals are computed against the same inflated AABBs as the collision checker, preventing rounded-distance/corner mismatches. | Green attraction, coral repulsion, purple escape and yellow command vectors |
| **DWA** | Track an original-route waypoint instead of pulling straight toward the final goal. Search acceleration-reachable XYZ velocities, predict 1.5 s of motion and require an admissible braking tail. | Good/bad rollouts, selected action, candidate count and altitude |
| **VFH 3D** | Use 24 azimuth × 13 elevation sectors, finite-radius clearance rays, angular opening checks and heading persistence. Track the original route, with upward/downward escape available. | Spherical direction rays and a two-dimensional azimuth/elevation histogram; white marks the selected sector |

Bug 3D, potential fields, DWA and VFH follow the **original unrepaired global polyline**, which intersects the newly detected load. They react locally to that obstruction; none calls A*, consumes a repaired detour or substitutes a successful path. Potential fields, DWA and VFH can finish directly if the final goal becomes visible. Bug keeps following ordered route samples even when the final goal is visible; samples are spaced at most 0.3 m apart, reached within 0.12 m, and those inside inflated obstacles are skipped. Yellow markers identify the current tracking/boundary target.

The potential-field escape term projects world-up onto the nearest obstacle face; over the top of a box it uses the projected target direction instead. This is an **augmented** field, not a claim that the original attractive/repulsive APF has no local minima. Bug uses finitely sampled surfaces and visit memory, without a general completeness guarantee. VFH uses idealized collision rays to populate a binary spherical histogram and opening checks, rather than the complete noisy-sensor 3DVFH+ implementation.

DWA limits velocity magnitude to 1.2 m/s and acceleration to 1.6 m/s² at a 0.15 s action interval; the cached executed controls are checked against both limits. The other three use bounded holonomic velocity commands without a quadrotor attitude/thrust model. A final segment check may scale an unsafe action; it does not generate an alternate route. Start/goal and obstacles remain unchanged.

Fresh **A* replanning** and incremental **D* Lite replanning** remain available separately. **MPC/MPPI** retain their 6/12/18-step horizons and 0.22 s control interval, using a separately validated detour guide. The new original-route local methods therefore have a different guidance assumption from MPC/MPPI; this is not a universal controller benchmark. MPPI is simplified weighted shooting with 64 samples; displayed MPC perturbations are alternatives, not optimizer iterations.

For 3D algorithm context, see [Frustumbug, TU Delft](https://repository.tudelft.nl/record/uuid%3A825121c5-b2c8-43ac-ac52-2672d837bc3b), [3D VFH documentation and 3DVFH+ reference](https://www.mathworks.com/help/uav/ref/controllervfh3d-system-object.html), and [augmented potential-field UAV navigation](https://arxiv.org/abs/2306.16276). These sources motivate the distinctions and adaptations; the code here is a small independent teaching implementation.

Velocity obstacles, ORCA and CBF remain possible extensions. The current new obstacle is stationary after detection.

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

`common/scene.js` supplies rendering/camera/colors; `common/math.js` supplies browser spatial operations; `common/algorithms.py`, `common/extensions.py` and `common/local_control.py` compute actual algorithm histories. Each topic owns a `view.js`; `app.js` owns shared controls/playback. `precomputed/lecture.json` stores histories, and `vendor/` contains offline Three.js r180 and its MIT notice. Source-derived data retains its original ownership and source terms.

Algorithm references: [D* Lite, Koenig and Likhachev](https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf), [Dynamic Window Approach, Fox, Burgard and Thrun](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance). Implementations here are deliberately small teaching examples; their specific adaptations are described above.
