export const topics={
 mapping:{label:'Map representations',eyebrow:'01 / UNDERSTAND SPACE',intro:'One environment. Many ways to represent it.',modes:{
 'Point cloud':['Sampled geometry','These are original hangar surface samples, spatially downsampled. A point is not a free-space measurement.'],
 'Dense voxels':['Allocate the whole volume','A fixed-resolution array reserves every cell, including empty space. Halving cell width requires about 8× the storage.'],
 'Sparse voxels':['Allocate only relevant cells','The very same occupied cells, indexed by a hash map. Fixed voxel size does not imply dense storage.'],
 'Voxel blocks':['Sparse globally · dense locally','Allocate 2 m blocks around a moving region of interest. Each block contains a dense local array.'],
 'Octree':['One cube → eight children','Subdivide cells containing surface samples; leave empty regions coarse. Hierarchy is a structure, not an occupancy value.'],
 'Occupancy':['Free · occupied · unknown','The planning model uses conservative solid rack envelopes. Free space here is known from the model, not inferred from missing PCD points.'],
 'Inflation':['Robot radius + safety margin','Expand forbidden space so the UAV can be treated as a point. Watch free gaps shrink as the radius grows.'],
 'Costmap':['Free does not mean equally desirable','A decaying clearance penalty favors routes away from obstacles. Green and purple show two feasible route choices.'],
 'ESDF':['Distance to the nearest obstacle','Signed Euclidean distance to the simplified solid scene. Click the horizontal slice to inspect clearance.' ]}},
 planning:{label:'Path planning',eyebrow:'02 / FIND A ROUTE',intro:'Same start. Same goal. Different search strategies.',modes:{
 'Dijkstra':['f(n) = g(n)','Expand by accumulated travel cost. Blue is visited; yellow is the actual frontier.'],
 'A*':['f(n) = g(n) + h(n)','A Euclidean lower bound guides a 6-connected 3D grid search toward the goal.'],
 'Weighted A*':['f(n) = g(n) + 2.5 h(n)','Stronger goal bias can reduce search effort, while giving up shortest-path guarantees.'],
 'Theta*':['Connect to a visible ancestor','During search, test the current node’s parent for a direct collision-free connection. Purple shows the A* baseline.'],
 'RRT':['Sample → nearest → extend','A seeded random tree grows through continuous 3D free space. Yellow marks the current random sample.'],
 'RRT*':['Choose parent → rewire','Nearby nodes adopt cheaper parents. Pink edges mark actual rewiring; the green best route improves as samples accumulate.'],
 'Informed RRT*':['Find a route → focus the samples','After a solution, sample inside the prolate ellipsoid that can contain a shorter path. Rewiring still improves the tree.'],
 'Comparison':['One controlled demonstration','Measured preparation runtimes for this scene and these implementations only. All routes use the same conservative swept-sphere collision test.']}},
 trajectory:{label:'Paths & trajectories',eyebrow:'03 / MAKE MOTION',intro:'A route says where. A trajectory also says when.',modes:{
 'Raw A*':['A geometric route','Every 0.5 m grid step is a waypoint. No timestamps or dynamics are specified.'],
 'Line-of-sight pruning':['Keep the farthest visible waypoint','Green candidate connections pass collision checks. Red ones fail. The final path retains only necessary corners.'],
 'Random shortcutting':['Try a shorter direct connection','Deterministically sample waypoint pairs and accept collision-free shortcuts.'],
 'Spline smoothing':['Smooth does not imply safe','A cubic B-spline approximates the control polygon and can cut into inflated obstacles. Red curve samples violate the clearance requirement.'],
 'Path vs trajectory':['p(t) → v(t) → a(t)','At a sharp corner, nonzero speed implies a discontinuous velocity. A timed stop-to-stop polynomial can slow down before turning.'],
 'Minimum snap':['min ∫ ‖p⁽⁴⁾(t)‖² dt','A joint seventh-degree polynomial solve enforces waypoint positions and continuity through jerk. Collision constraints are not part of this solve.'],
 'Minimum jerk':['min ∫ ‖p⁽³⁾(t)‖² dt','A joint fifth-degree polynomial solve enforces waypoint positions and continuity through acceleration.'],
 'CHOMP':['Smoothness + distance-gradient descent','A simplified covariant optimizer moves interior waypoints using smoothness and obstacle gradients. Feasible updates are accepted with a line search.']}},
 avoidance:{label:'Local avoidance',eyebrow:'04 / REACT TO CHANGE',intro:'Global intention. Local decisions. Executed motion.',modes:{
 'A* replanning':['Detect → inflate → replan','An initially unknown suspended load intersects the global route. Update the map, then run A* again. Predictive control is not mandatory.'],
 'Potential field':['Attraction + repulsion','Cyan attracts toward the goal; coral repels from obstacles. The U-shaped example exposes a local minimum.'],
 'MPC':['Predict → optimize → execute one action','Bounded 3D velocity controls use a finite horizon. Execute only the first action, shift the horizon, and solve again.'],
 'MPPI':['Sample → score → weight → execute','64 noisy control sequences are scored. Their exponential weights produce the next command; 24 rollouts are displayed.']}},
 full_pipeline:{label:'Complete pipeline',eyebrow:'05 / CONNECT THE IDEAS',intro:'From sampled surfaces to a drone reaching its goal.',modes:{'Full lecture demo':['The complete pipeline','A prepared, deterministic sequence connects mapping, inflation, search, path processing and obstacle response.']}}
};
export const stages=['Hangar geometry','Point cloud','Voxel occupancy','Inflation','A* search','Global path','Path pruning','Timed motion','Unknown obstacle','Replanning','Drone execution','Goal reached'];
