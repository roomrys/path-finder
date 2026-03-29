'use strict';

/* ============================================================
   js/sampling-algorithms.js
   Sampling-based motion-planning algorithms.
   Each class extends SamplingAlgorithm and self-registers.

   run(env) -> { steps: Step[], path: Point[]|null }
     env  = { width, height, obstacles, start, end }
     Step = { type:'edge', subtype:'tree'|'roadmap', x1,y1,x2,y2 }
          | { type:'path', points:[{x,y}...] }
   ============================================================ */

// ---------------------------------------------------------------------------
// RRT – Rapidly-exploring Random Tree
// ---------------------------------------------------------------------------
class RRT extends SamplingAlgorithm {
  static id          = 'rrt';
  static displayName = 'RRT';
  static description = 'Rapidly-exploring Random Tree. Grows a tree by sampling the space at random and steering toward samples. Probabilistically complete.';
  static paperUrl    = 'http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf';
  static paperTitle  = 'Rapidly-Exploring Random Trees: A New Tool for Path Planning (LaValle, 1998)';
  static pseudocode  =
`T <- tree rooted at start

Repeat until goal reached or MAX_ITER exceeded:
  q_rand <- random point  (goal with prob GOAL_BIAS)
  q_near <- nearest node in T to q_rand
  q_new  <- steer(q_near -> q_rand, step = STEP_SIZE)

  If q_new is collision-free and edge(q_near, q_new) is clear:
    Add q_new to T,  parent(q_new) <- q_near

    If dist(q_new, goal) < GOAL_THRESHOLD and edge is clear:
      Connect q_new -> goal
      Trace parents: goal -> ... -> start
      Return path

Return  failure  (no path found in MAX_ITER iterations)`;

  static MAX_ITER       = 3000;
  static STEP_SIZE      = 25;
  static GOAL_THRESHOLD = 20;
  static GOAL_BIAS      = 0.05;

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1 }];
    const steps = [];

    for (let i = 0; i < this.MAX_ITER; i++) {
      // Sample – occasionally bias toward goal
      const qRand = Math.random() < this.GOAL_BIAS
        ? { x: end.x, y: end.y }
        : this.randomPoint(width, height);

      // Nearest node (squared-distance scan)
      let nearIdx = 0, nearD2 = Infinity;
      for (let j = 0; j < nodes.length; j++) {
        const d2 = (nodes[j].x - qRand.x) ** 2 + (nodes[j].y - qRand.y) ** 2;
        if (d2 < nearD2) { nearD2 = d2; nearIdx = j; }
      }
      const qNear = nodes[nearIdx];
      const qNew  = this.steer(qNear, qRand, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles) || !this.edgeFree(qNear, qNew, obstacles)) continue;

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: qNear.x, y1: qNear.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx });
        steps.push({ type: 'edge', subtype: 'tree',
          x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });

        const path = [];
        let cur = nodes.length - 1;
        while (cur !== -1) {
          path.unshift({ x: nodes[cur].x, y: nodes[cur].y });
          cur = nodes[cur].parentIdx;
        }
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(RRT);

// ---------------------------------------------------------------------------
// RRT* – Asymptotically Optimal RRT
// ---------------------------------------------------------------------------
class RRTStar extends SamplingAlgorithm {
  static id          = 'rrt-star';
  static displayName = 'RRT*';
  static description = 'Asymptotically optimal extension of RRT. Continuously rewires the tree to minimise path cost, converging toward the global optimum.';
  static paperUrl    = 'https://doi.org/10.1177/0278364911406761';
  static paperTitle  = 'Sampling-based Algorithms for Optimal Motion Planning (Karaman & Frazzoli, 2011)';
  static pseudocode  =
`T <- tree rooted at start,  cost[start] <- 0

For i = 1 to MAX_ITER:
  q_rand <- random point in free space
  q_near <- nearest node in T to q_rand
  q_new  <- steer(q_near -> q_rand, step = STEP_SIZE)

  If q_new is collision-free and edge is clear:
    Q_near <- { n in T | dist(n, q_new) <= NEIGHBOR_RADIUS, edge clear }

    // Choose minimum-cost parent
    q_min <- argmin over Q_near of  cost[q] + dist(q, q_new)
    Add q_new to T with parent q_min
    cost[q_new] <- cost[q_min] + dist(q_min, q_new)

    // Rewire neighbourhood
    For each q in Q_near:
      If cost[q_new] + dist(q_new, q) < cost[q]:
        If edge(q_new, q) is obstacle-free:
          Re-parent q to q_new;  update cost[q]

    Track best connection to goal

Return best path found  (or failure if goal never reached)`;

  static MAX_ITER        = 2000;
  static STEP_SIZE       = 25;
  static GOAL_THRESHOLD  = 20;
  static NEIGHBOR_RADIUS = 55;

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1, cost: 0 }];
    const steps = [];
    let bestGoalParentIdx = -1;
    let bestGoalCost      = Infinity;

    for (let i = 0; i < this.MAX_ITER; i++) {
      const qRand = this.randomPoint(width, height);

      // Nearest node
      let nearIdx = 0, nearD2 = Infinity;
      for (let j = 0; j < nodes.length; j++) {
        const d2 = (nodes[j].x - qRand.x) ** 2 + (nodes[j].y - qRand.y) ** 2;
        if (d2 < nearD2) { nearD2 = d2; nearIdx = j; }
      }
      const qNew = this.steer(nodes[nearIdx], qRand, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles) || !this.edgeFree(nodes[nearIdx], qNew, obstacles)) continue;

      // Collect obstacle-free neighbours within NEIGHBOR_RADIUS
      const nbrIdxs = [];
      for (let j = 0; j < nodes.length; j++) {
        if (this.dist(nodes[j], qNew) <= this.NEIGHBOR_RADIUS &&
            this.edgeFree(nodes[j], qNew, obstacles)) {
          nbrIdxs.push(j);
        }
      }

      // Choose minimum-cost parent from neighbours (fallback: nearest)
      let minParent = nearIdx;
      let minCost   = nodes[nearIdx].cost + this.dist(nodes[nearIdx], qNew);
      for (const j of nbrIdxs) {
        const c = nodes[j].cost + this.dist(nodes[j], qNew);
        if (c < minCost) { minCost = c; minParent = j; }
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: minParent, cost: minCost });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: nodes[minParent].x, y1: nodes[minParent].y, x2: qNew.x, y2: qNew.y });

      // Rewire: re-parent neighbours if cheaper through q_new
      for (const j of nbrIdxs) {
        const rewireCost = nodes[newIdx].cost + this.dist(nodes[newIdx], nodes[j]);
        if (rewireCost < nodes[j].cost) {
          nodes[j].parentIdx = newIdx;
          nodes[j].cost      = rewireCost;
          steps.push({ type: 'edge', subtype: 'tree',
            x1: qNew.x, y1: qNew.y, x2: nodes[j].x, y2: nodes[j].y });
        }
      }

      // Track best goal connection found so far
      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        const goalCost = nodes[newIdx].cost + this.dist(qNew, end);
        if (goalCost < bestGoalCost) {
          bestGoalCost      = goalCost;
          bestGoalParentIdx = newIdx;
        }
      }
    }

    if (bestGoalParentIdx === -1) return { steps, path: null };

    nodes.push({ x: end.x, y: end.y, parentIdx: bestGoalParentIdx, cost: bestGoalCost });
    steps.push({ type: 'edge', subtype: 'tree',
      x1: nodes[bestGoalParentIdx].x, y1: nodes[bestGoalParentIdx].y,
      x2: end.x, y2: end.y });

    const path = [];
    let cur = nodes.length - 1;
    while (cur !== -1) {
      path.unshift({ x: nodes[cur].x, y: nodes[cur].y });
      cur = nodes[cur].parentIdx;
    }
    steps.push({ type: 'path', points: path });
    return { steps, path };
  }
}
Algorithm.register(RRTStar);

// ---------------------------------------------------------------------------
// PRM – Probabilistic Roadmap Method
// ---------------------------------------------------------------------------
class PRM extends SamplingAlgorithm {
  static id          = 'prm';
  static displayName = 'PRM';
  static description = 'Probabilistic Roadmap Method. Samples the free space to build a reusable roadmap, then queries it with a graph search to find paths.';
  static paperUrl    = 'https://doi.org/10.1109/70.508439';
  static paperTitle  = 'Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces (Kavraki et al., 1996)';
  static pseudocode  =
`-- Learning Phase --
For i = 1 to N_SAMPLES:
  q <- random free configuration
  Add q to roadmap R

  For each node n in R within CONNECT_RADIUS of q:
    If edge(q, n) is obstacle-free:
      Add bidirectional edge (q <-> n) to R

-- Query Phase --
Add start and goal to R using the same connection rule

Run BFS on roadmap R from start to goal
If goal is reachable: trace parents and return path
Else: return failure`;

  static N_SAMPLES      = 300;
  static CONNECT_RADIUS = 80;

  static run({ width, height, obstacles, start, end }) {
    const steps = [];
    // Node 0 = start, node 1 = end, nodes 2+ = samples
    const nodes = [{ x: start.x, y: start.y }, { x: end.x, y: end.y }];

    for (let i = 0; i < this.N_SAMPLES; i++) {
      nodes.push(this.randomFreePoint(width, height, obstacles));
    }

    // Build adjacency list
    const adj = Array.from({ length: nodes.length }, () => []);
    for (let i = 0; i < nodes.length; i++) {
      for (let j = i + 1; j < nodes.length; j++) {
        if (this.dist(nodes[i], nodes[j]) <= this.CONNECT_RADIUS &&
            this.edgeFree(nodes[i], nodes[j], obstacles)) {
          adj[i].push(j);
          adj[j].push(i);
          steps.push({ type: 'edge', subtype: 'roadmap',
            x1: nodes[i].x, y1: nodes[i].y, x2: nodes[j].x, y2: nodes[j].y });
        }
      }
    }

    // BFS from start (index 0) to goal (index 1)
    const visited = new Array(nodes.length).fill(false);
    const parent  = new Array(nodes.length).fill(-1);
    const queue   = [0];
    visited[0] = true;

    while (queue.length > 0) {
      const cur = queue.shift();
      if (cur === 1) break;
      for (const next of adj[cur]) {
        if (!visited[next]) {
          visited[next] = true;
          parent[next]  = cur;
          queue.push(next);
        }
      }
    }

    if (!visited[1]) return { steps, path: null };

    const path = [];
    let cur = 1;
    while (cur !== -1) {
      path.unshift({ x: nodes[cur].x, y: nodes[cur].y });
      cur = parent[cur];
    }
    steps.push({ type: 'path', points: path });
    return { steps, path };
  }
}
Algorithm.register(PRM);
