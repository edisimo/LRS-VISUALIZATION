# A lecture through one hangar

Start the lab once with `./run_full_demo.sh`. All topics remain in the same browser tab. The 3D planning views use the same solid shelving envelopes, start, goal and collision radius. The point cloud shows the original hangar, and the app labels the transition to simplified occupancy.

## Very short route · 6–8 minutes

1. **Mapping → Point cloud** (30 s): orbit once. “Surface samples do not tell us that unsampled space is free.”
2. **Dense → Sparse → Octree** (60 s): identical geometry can have different storage structures. Change cell width from 1 to 0.5 m; distinguish cell count from actual occupied count.
3. **Inflation → ESDF** (60 s): increase radius and margin until the aisle closes. Click a distance slice. “Robot size affects collision space; clearance is information we can store in that map.”
4. **Planning → Dijkstra → A* → Theta*** (60 s): scrub each to the end, compare expansions, then compare grid and any-angle paths.
5. **RRT* → Informed RRT*** (45 s): play at 2×. Highlight pink rewires and the ellipsoid after the first solution.
6. **Trajectory → Path vs trajectory → Minimum snap** (60 s): show the stop at the sharp corner, then inspect the timed derivatives and discuss why collision checks are still needed. Smoothness and safety are separate requirements.
7. **Avoidance → MPPI** (45 s): purple global route, candidate rollouts, green prediction, white actual motion. Step once: only the first command executes.
8. **Complete pipeline** (42 s at 2×): narrate the connections while the sequence plays through arrival.

## Longer route · 18–22 minutes

1. **PCD and bounded crop** (1 min): introduce the real asset and the simplified conservative shelving model.
2. **Dense / sparse / blocks / octree** (3 min): storage, allocation overhead, rolling local region, eight children per split. Sparse does not automatically mean lower memory.
3. **Occupancy / inflation / cost / ESDF** (3 min): distinguish stored information from its data structure. Show the aisle-width readout and probe clearance at different altitudes.
4. **Dijkstra / A* / weighted A* / Theta*** (2 min): same graph and collision model, different priorities and parent connections.
5. **RRT / RRT* / Informed RRT* / comparison** (3 min): feasible route versus improving route; sampling focus after an incumbent solution. Mention that preparation timings are scene-specific.
6. **Pruning / random shortcutting / spline** (2 min): pause on a rejected shortcut, then show a smooth curve cutting a forbidden region.
7. **Path versus trajectory / minimum snap / CHOMP** (3 min): timestamps and derivatives; unconstrained polynomial smoothing versus distance-driven geometry optimization. Watch the CHOMP objective decrease.
8. **Replanning / potential-field failure / MPC / MPPI** (3 min): a controller is not mandatory for avoidance. Explain the cached detour-guided, single-integrator model. Compare prediction horizons and step the receding horizon.
9. **Full pipeline** (84 s at 1×): consolidate the concepts.

## Before students arrive

Open the application once in the presentation browser. Confirm orbit and zoom work, select fullscreen, and set the desired playback speed. All assets are local; regeneration is not needed. Keep the CLI server running throughout the lecture. Camera reset is **C**, demo reset is **R**, pause is **Space**, and **1–5** switch topics when form controls are not focused.

The unknown obstacle is a newly detected stationary suspended load. The full pipeline uses A* repair plus validated stop-to-stop motion. The separate spline demonstration shows an actual collision; the minimum-snap demonstration reports the clearance of its unconstrained smoothness optimum; do not describe it as a certified executable flight trajectory.
