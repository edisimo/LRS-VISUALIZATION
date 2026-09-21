# Suggested lecture order

Start any launcher once, then move between the four tabs. No preprocessing is needed.

## Short route · 7–9 minutes

1. **Point cloud → Dense → Sparse** (1 min). Start at the default 25 cm, then show 10 cm cells and the active memory estimates. Sparse stores fewer cells but pays indexing overhead; switch to 50 cm to see how the trade-off changes.
2. **Octree → Mesh → Elevation map** (1 min). Depth controls hierarchical resolution. Meshes store surfaces; a single-height map cannot retain all the free space under shelves.
3. **Inflation** (1 min). Show whole-volume inflation first, then select a slice, increase robot radius, and count extra layers. Switch to octree: differently sized leaves prevent a single universal layer count.
4. **Occupancy → Costmap → ESDF** (1 min). Click one free cell beside a rack and retain it across all three views: “free”, “expensive”, and “0.6 metres away” answer different questions. No paths are involved.
5. **A* → Theta* → Comparison** (1 min). Both use 26 neighbours. Toggle routes individually in Comparison; discuss ancestor connections separately from graph connectivity.
6. **PRM → D* Lite** (1 min). A reusable roadmap versus an initial backward search. Show reuse of search state after an obstacle update in Local avoidance.
7. **Pruning → STOMP** (1 min). Accepted/rejected shortcuts, then weighted noisy improvements.
8. **Potential field → DWA → MPPI** (1–2 min). Same obstacles, different decisions. Watch the revised methods reach the goal with vertical motion. Explain original-route local tracking versus MPC/MPPI detour guidance.

## Longer route · 18–22 minutes

- Spend 4 minutes on geometry/storage: PCD, dense/hash memory, blocks, octree depths 1–6, meshes and elevation limits.
- Spend 3 minutes on obstacle-only inflation and the shared occupancy/cost/distance probe.
- Spend 5 minutes on Dijkstra/A*/weighted A*/Theta*, RRT rewiring/informed sampling, PRM construction and D* Lite repairs. Use comparison checkboxes to isolate pairs.
- Spend 4 minutes on pruning, spline collision, minimum snap/jerk, CHOMP gradients and STOMP perturbations. The processing baseline is deliberately stepped 6-neighbour A*, while the planner comparison uses 26.
- Spend 4–6 minutes on fresh versus incremental replanning, tangential potential-field escape in the rack scene, 3D Bug boundary samples, spherical VFH sectors, 3D DWA and receding-horizon MPC/MPPI.

There is no Complete pipeline section or Path vs trajectory vignette. Use the four tabs to connect the concepts verbally.

Before the lecture, check orbit/zoom in the presentation browser, enter fullscreen and choose playback speed. Keep the local server running. R resets the current demo, C resets the camera, Space pauses and 1–4 select topics when form controls are not focused.
