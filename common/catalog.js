export const topics={
 mapping:{label:'Map representations',eyebrow:'01 / REPRESENT SPACE',intro:'First: how we store space. Then: what each location tells us.',modes:{
 'Point cloud':['Geometry · {p₁, p₂, …}','Measured surface samples. Points alone do not tell us which unsampled regions are free.'],
 'Dense voxels':['Storage · M = N × bytes per cell','Allocate the whole 3D array. The active memory estimate includes empty cells; the faint floor lattice illustrates allocation.'],
 'Sparse voxels':['Storage · M ≈ K × bytes per entry','Allocate only K relevant cells, rather than all N cells. A hash entry needs coordinates and indexing overhead as well as its value.'],
 'Voxel blocks':['Storage · sparse outside, dense inside','A sparse directory locates 2 m blocks. Each allocated block contains a dense voxel array.'],
 'Octree':['Storage · one parent → eight children','Refine cells near surface samples. Increasing depth gives smaller leaves; empty regions remain coarse.'],
 'Triangle mesh':['Geometry · vertices + triangle faces','Connected triangles describe surfaces directly. Here the rack envelopes become closed meshes, without allocating a volume of cells.'],
 'Elevation map':['Geometry · z = h(x, y)','One maximum observed height per XY column. Compact 2.5D storage loses stacked surfaces and free space under shelves: a limitation for indoor UAVs.'],
 'Inflation':['Collision space · n = ceil((r + margin) / Δ)','Only obstacles grow. Compare drone size with voxel width and count the extra forbidden cell layers, or inspect adaptive octree leaves.'],
 'Occupancy':['1 / Is this cell occupied?','A yes/no value at each location. Coral means obstacle; green means known free. Click a location, then switch to Costmap or ESDF to inspect that same cell.'],
 'Costmap':['2 / How costly is this cell?','Free locations need not be equally desirable. C = 100 exp(−max(d,0)/falloff): nearby obstacles raise the penalty, even outside occupied space.'],
 'ESDF':['3 / How far is the nearest surface?','Store signed distance in metres, not a yes/no label or arbitrary penalty. Click to reveal a ruler from the selected cell to its nearest rack surface.']}},
 planning:{label:'Path planning',eyebrow:'02 / FIND A ROUTE',intro:'One collision model. Shared 26-neighbour graph. Different search logic.',modes:{
 'Dijkstra':['f(n) = g(n)','Expand the smallest accumulated path cost. No estimate of the remaining distance; 26 neighbours and Euclidean edge costs.'],
 'A*':['f(n) = g(n) + h(n)','Use travelled cost g plus Euclidean distance-to-go h. All 26 adjacent cells are considered; every diagonal edge is collision-checked.'],
 'Weighted A*':['f(n) = g(n) + 2.5 h(n)','A stronger goal bias trades shortest-path guarantees for potentially fewer expansions on the same 26-neighbour graph.'],
 'Theta*':['g(v) ← min[g(u)+c(u,v), g(parent(u))+c(parent(u),v)]','Use the same 26 neighbours as A*. Also connect to a visible ancestor, so the route is not restricted to grid edges.'],
 'D* Lite':['rhs(u) = minᵥ[c(u,v) + g(v)]','Reuse g and one-step lookahead rhs. After costs change, repair inconsistent vertices (g ≠ rhs), rather than discard the previous search.'],
 'RRT':['qnew = qnear + η · direction(qsample − qnear)','Sample free space, find the nearest vertex and extend by at most η. Connect only after a collision test.'],
 'RRT*':['parent(q) = argminᵥ[g(v) + ‖q−v‖]','Choose the cheapest visible neighbour, then rewire nearby vertices if their accumulated cost improves. Pink highlights real rewiring events.'],
 'Informed RRT*':['‖q−start‖ + ‖q−goal‖ < cbest','Before a solution, sample globally. Afterwards sample inside the improving ellipsoid; retain RRT* parent selection and rewiring.'],
 'PRM':['Sample → connect k visible neighbours → graph search','Build a collision-free roadmap before searching it. Unlike RRT, the graph does not grow from just the start, and it can support repeated queries.'],
 'Comparison':['Same initial map · choose the visible routes','Toggle any planner. D* Lite shows its initial route here; its obstacle-update repair is a separate sequence. Preparation timings are scene-specific.']}},
 trajectory:{label:'Paths & trajectories',eyebrow:'03 / REFINE THE ROUTE',intro:'Remove unnecessary points, smooth motion, and respect obstacles.',modes:{
 'Raw A*':['Grid waypoints · geometric route','The deliberately stepped 6-neighbour A* baseline is retained here for processing demonstrations. Planner comparisons use 26 neighbours.'],
 'Line-of-sight pruning':['Keep the farthest collision-free connection','Test distant waypoints; accept green connections and reject red ones.'],
 'Random shortcutting':['Accept if collision-free and shorter','Sample waypoint pairs and replace the intervening polyline by a validated segment.'],
 'Spline smoothing':['p(u) = Σᵢ Nᵢ,₃(u) Pᵢ','A cubic B-spline approximates the control polygon. Smoothness alone does not ensure clearance: red samples collide.'],
 'Minimum snap':['min ∫ ‖p⁽⁴⁾(t)‖² dt','Seventh-degree polynomials satisfy waypoint and continuity constraints through jerk. Collision and actuator limits are not constraints of this solve.'],
 'Minimum jerk':['min ∫ ‖p⁽³⁾(t)‖² dt','Fifth-degree polynomials satisfy waypoints and continuity through acceleration, with fixed segment times.'],
 'CHOMP':['q ← q − α A⁻¹ ∇J(q)','Covariant gradient descent balances smoothness with obstacle-distance penalties. Orange shows distance ascent; updates retain collision feasibility.'],
 'STOMP':['wₖ ∝ exp(−β J(q + εₖ)); Δq = Σₖ wₖ εₖ','Try correlated noisy trajectories, score them, and combine perturbations with cost-based weights. No obstacle-cost gradient is required.']}},
 avoidance:{label:'Local avoidance',eyebrow:'04 / REACT TO CHANGE',intro:'The same racks, start, goal and newly detected obstacle.',modes:{
 'A* replanning':['Changed map → fresh A* search','Inflate the new obstacle and search again. The UAV waits while the global route is replaced.'],
 'D* Lite replanning':['Changed edges → repair g ≠ rhs','Keep the previous search state and propagate only the necessary consistency updates. This is incremental global replanning, not a reactive controller.'],
 'Potential field':['v ∝ −∇(Uattractive + Urepulsive)','Attraction and repulsion act in the same rack scene as the other methods. Local minima and oscillation are possible; the observed outcome is shown.'],
 'Bug2':['Seek goal → follow boundary → rejoin m-line','Classic 2D logic at fixed altitude: leave an obstacle at a closer crossing of the start–goal line. A teaching adaptation, not a full 3D flight planner.'],
 'DWA':['v ∈ reachable window; reject unsafe braking trajectories','Sample acceleration-reachable 3D velocities, predict short rollouts, check stopping clearance, then score goal progress. A holonomic UAV adaptation of DWA.'],
 'VFH':['Polar histogram → free sector → steering direction','Threshold local obstacle directions and steer through an open angular valley. This fixed-altitude binary VFH illustration cannot exploit vertical escape.'],
 'MPC':['u* = argmin J(u₀…uH−1); execute u₀','Optimize bounded velocity controls over a finite horizon, apply the first action and solve again. A detour provides a terminal guide.'],
 'MPPI':['wₖ ∝ exp(−Jₖ/λ); u ← Σₖ wₖ uₖ','Score 64 noisy control sequences and weight the controls. Apply one command, shift the horizon and repeat. The detour supplies guidance.']}}
};
export const comparisonNames=['Dijkstra','A*','Weighted A*','Theta*','D* Lite','RRT','RRT*','Informed RRT*','PRM'];
