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

// ---------------------------------------------------------------------------
// RRTConnect – Bidirectional RRT
// ---------------------------------------------------------------------------
class RRTConnect extends SamplingAlgorithm {
  static id          = 'rrt-connect';
  static displayName = 'RRTConnect';
  static description = 'Bidirectional RRT variant. Grows two trees (from start and goal) and greedily attempts to connect them.';
  static paperUrl    = 'http://www.kuffner.org/james/papers/kuffner_icra2000.pdf';
  static paperTitle  = 'RRT-Connect: An Efficient Approach to Single-Query Path Planning (Kuffner & LaValle, 2000)';
  static pseudocode  =
`T_a <- tree rooted at start
T_b <- tree rooted at goal

Repeat until MAX_ITER:
  q_rand <- random sample
  q_new  <- EXTEND(T_a, q_rand)
  If q_new added:
    If CONNECT(T_b, q_new) succeeds:
      Return merged path(start -> ... -> goal)
  Swap T_a and T_b

Return failure`;

  static MAX_ITER  = 2200;
  static STEP_SIZE = 25;

  static _nearestIdx(nodes, target) {
    let bestIdx = 0;
    let bestD2  = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d2 = (nodes[i].x - target.x) ** 2 + (nodes[i].y - target.y) ** 2;
      if (d2 < bestD2) { bestD2 = d2; bestIdx = i; }
    }
    return bestIdx;
  }

  static _extend(nodes, target, obstacles, steps) {
    const nearIdx = this._nearestIdx(nodes, target);
    const near    = nodes[nearIdx];
    const qNew    = this.steer(near, target, this.STEP_SIZE);

    if (!this.isFree(qNew, obstacles) || !this.edgeFree(near, qNew, obstacles)) return -1;

    const newIdx = nodes.length;
    nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx });
    steps.push({ type: 'edge', subtype: 'tree',
      x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
    return newIdx;
  }

  static _connect(nodes, target, obstacles, steps) {
    let lastIdx = -1;
    while (true) {
      const nearIdx = this._nearestIdx(nodes, target);
      const near    = nodes[nearIdx];
      const d       = this.dist(near, target);
      const qNew    = this.steer(near, target, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles) || !this.edgeFree(near, qNew, obstacles)) break;

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
      lastIdx = newIdx;

      if (d <= this.STEP_SIZE) return { connected: true, idx: newIdx };
    }
    return { connected: false, idx: lastIdx };
  }

  static _trace(nodes, idx) {
    const out = [];
    let cur = idx;
    while (cur !== -1) {
      out.push({ x: nodes[cur].x, y: nodes[cur].y });
      cur = nodes[cur].parentIdx;
    }
    return out; // from node back to root
  }

  static run({ width, height, obstacles, start, end }) {
    const steps = [];

    let treeA = [{ x: start.x, y: start.y, parentIdx: -1 }];
    let treeB = [{ x: end.x, y: end.y, parentIdx: -1 }];
    let aFromStart = true;

    for (let i = 0; i < this.MAX_ITER; i++) {
      const qRand = this.randomPoint(width, height);
      const newAIdx = this._extend(treeA, qRand, obstacles, steps);
      if (newAIdx !== -1) {
        const qNew = treeA[newAIdx];
        const connectRes = this._connect(treeB, qNew, obstacles, steps);

        if (connectRes.connected) {
          const aBranch = this._trace(treeA, newAIdx).reverse(); // root->join
          const bBranch = this._trace(treeB, connectRes.idx);    // join->root
          const path = aFromStart ? aBranch.concat(bBranch) : bBranch.reverse().concat(aBranch.reverse());
          steps.push({ type: 'path', points: path });
          return { steps, path };
        }
      }

      // Swap trees every iteration
      const temp = treeA;
      treeA = treeB;
      treeB = temp;
      aFromStart = !aFromStart;
    }

    return { steps, path: null };
  }
}
Algorithm.register(RRTConnect);

// ---------------------------------------------------------------------------
// InformedRRT* – informed subset sampling once a solution exists
// ---------------------------------------------------------------------------
class InformedRRTStar extends SamplingAlgorithm {
  static id          = 'informed-rrt-star';
  static displayName = 'InformedRRT*';
  static description = 'RRT* variant that samples from an informed ellipsoid after finding an initial path.';
  static paperUrl    = 'https://arxiv.org/abs/1404.2334';
  static paperTitle  = 'Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic (Gammell et al., 2014)';
  static pseudocode  =
`Run RRT* until a first solution exists
Then sample from the informed ellipsoid connecting start/goal
Continue rewiring to reduce solution cost`;

  static MAX_ITER        = 2500;
  static STEP_SIZE       = 24;
  static GOAL_THRESHOLD  = 18;
  static NEIGHBOR_RADIUS = 60;
  static GOAL_BIAS       = 0.07;

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1, cost: 0 }];
    const steps = [];
    let bestGoalParentIdx = -1;
    let bestGoalCost = Infinity;

    for (let i = 0; i < this.MAX_ITER; i++) {
      let qRand;
      if (Math.random() < this.GOAL_BIAS) {
        qRand = { x: end.x, y: end.y };
      } else if (bestGoalParentIdx !== -1) {
        qRand = this.informedSample(start, end, bestGoalCost, width, height, obstacles);
      } else {
        qRand = this.randomFreePoint(width, height, obstacles);
      }

      const nearIdx = this.nearestIdx(nodes, qRand);
      const qNew = this.steer(nodes[nearIdx], qRand, this.STEP_SIZE);
      if (!this.isFree(qNew, obstacles) || !this.edgeFree(nodes[nearIdx], qNew, obstacles)) continue;

      const nbrIdxs = [];
      for (let j = 0; j < nodes.length; j++) {
        if (this.dist(nodes[j], qNew) <= this.NEIGHBOR_RADIUS &&
            this.edgeFree(nodes[j], qNew, obstacles)) {
          nbrIdxs.push(j);
        }
      }
      if (nbrIdxs.length === 0) nbrIdxs.push(nearIdx);

      let minParent = nbrIdxs[0];
      let minCost = nodes[minParent].cost + this.dist(nodes[minParent], qNew);
      for (const j of nbrIdxs) {
        const c = nodes[j].cost + this.dist(nodes[j], qNew);
        if (c < minCost) { minCost = c; minParent = j; }
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: minParent, cost: minCost });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: nodes[minParent].x, y1: nodes[minParent].y, x2: qNew.x, y2: qNew.y });

      for (const j of nbrIdxs) {
        const rewireCost = nodes[newIdx].cost + this.dist(nodes[newIdx], nodes[j]);
        if (rewireCost + 1e-6 < nodes[j].cost && this.edgeFree(nodes[newIdx], nodes[j], obstacles)) {
          nodes[j].parentIdx = newIdx;
          nodes[j].cost = rewireCost;
          steps.push({ type: 'edge', subtype: 'tree',
            x1: qNew.x, y1: qNew.y, x2: nodes[j].x, y2: nodes[j].y });
        }
      }

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        const goalCost = nodes[newIdx].cost + this.dist(qNew, end);
        if (goalCost < bestGoalCost) {
          bestGoalCost = goalCost;
          bestGoalParentIdx = newIdx;
        }
      }
    }

    if (bestGoalParentIdx === -1) return { steps, path: null };
    nodes.push({ x: end.x, y: end.y, parentIdx: bestGoalParentIdx, cost: bestGoalCost });
    steps.push({ type: 'edge', subtype: 'tree',
      x1: nodes[bestGoalParentIdx].x, y1: nodes[bestGoalParentIdx].y, x2: end.x, y2: end.y });
    const path = this.tracePath(nodes, nodes.length - 1);
    steps.push({ type: 'path', points: path });
    return { steps, path };
  }
}
Algorithm.register(InformedRRTStar);

// ---------------------------------------------------------------------------
// TRRT – transition-based acceptance on a simple obstacle-clearance cost map
// ---------------------------------------------------------------------------
class TRRT extends SamplingAlgorithm {
  static id          = 'trrt';
  static displayName = 'TRRT';
  static description = 'Transition-based RRT that accepts uphill moves probabilistically based on a temperature schedule.';
  static paperUrl    = 'https://doi.org/10.1109/IROS.2008.4650993';
  static paperTitle  = 'Transition-based RRT for Path Planning in Continuous Cost Spaces (Jaillet, Cortes, Simeon, 2008)';
  static pseudocode  =
`Grow an RRT
Accept transitions to higher-cost states with probability exp(-delta/T)
Adapt temperature as transitions are accepted/rejected`;

  static MAX_ITER = 2600;
  static STEP_SIZE = 22;
  static GOAL_THRESHOLD = 20;
  static GOAL_BIAS = 0.05;
  static T_INIT = 0.12;

  static stateCost(p, obstacles) {
    let minClearance = Infinity;
    for (const o of obstacles) {
      const clearance = Math.sqrt((p.x - o.cx) ** 2 + (p.y - o.cy) ** 2) - o.r;
      if (clearance < minClearance) minClearance = clearance;
    }
    if (!Number.isFinite(minClearance)) minClearance = 60;
    return 1 / (Math.max(0, minClearance) + 1);
  }

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1, cost: this.stateCost(start, obstacles) }];
    const steps = [];
    let temperature = this.T_INIT;

    for (let i = 0; i < this.MAX_ITER; i++) {
      const qRand = Math.random() < this.GOAL_BIAS
        ? { x: end.x, y: end.y }
        : this.randomFreePoint(width, height, obstacles);
      const nearIdx = this.nearestIdx(nodes, qRand);
      const near = nodes[nearIdx];
      const qNew = this.steer(near, qRand, this.STEP_SIZE);
      if (!this.isFree(qNew, obstacles) || !this.edgeFree(near, qNew, obstacles)) continue;

      const cNear = this.stateCost(near, obstacles);
      const cNew = this.stateCost(qNew, obstacles);
      const delta = cNew - cNear;
      const accept = delta <= 0 || Math.random() < Math.exp(-delta / Math.max(temperature, 1e-5));
      if (!accept) {
        temperature *= 1.02;
        continue;
      }
      temperature *= 0.995;

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx, cost: cNew });
      steps.push({ type: 'edge', subtype: 'tree', x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx, cost: 0 });
        steps.push({ type: 'edge', subtype: 'tree', x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(TRRT);

// ---------------------------------------------------------------------------
// EST – expansive space tree using density-biased node expansion
// ---------------------------------------------------------------------------
class EST extends SamplingAlgorithm {
  static id          = 'est';
  static displayName = 'EST';
  static description = 'Expansive Space Trees planner with preference for low-density frontier nodes.';
  static paperUrl    = 'https://doi.org/10.1142/S0218195999000285';
  static paperTitle  = 'Path Planning in Expansive Configuration Spaces (Hsu, Latombe, Motwani, 1999)';
  static pseudocode  =
`Pick nodes in low-density regions more often
Sample short random motions from picked nodes
Add collision-free successors to the tree`;

  static MAX_ITER = 2600;
  static STEP_SIZE = 22;
  static DENSITY_RADIUS = 55;
  static GOAL_THRESHOLD = 20;

  static weightedIndex(weights) {
    const total = weights.reduce((a, b) => a + b, 0);
    let r = Math.random() * total;
    for (let i = 0; i < weights.length; i++) {
      r -= weights[i];
      if (r <= 0) return i;
    }
    return weights.length - 1;
  }

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1 }];
    const steps = [];

    for (let i = 0; i < this.MAX_ITER; i++) {
      const density = new Array(nodes.length).fill(0);
      for (let a = 0; a < nodes.length; a++) {
        for (let b = a + 1; b < nodes.length; b++) {
          if (this.dist(nodes[a], nodes[b]) < this.DENSITY_RADIUS) {
            density[a] += 1;
            density[b] += 1;
          }
        }
      }
      const weights = density.map(d => 1 / (1 + d));
      const idx = this.weightedIndex(weights);
      const from = nodes[idx];

      const angle = Math.random() * Math.PI * 2;
      const target = {
        x: Math.max(0, Math.min(width, from.x + Math.cos(angle) * this.STEP_SIZE * 1.8)),
        y: Math.max(0, Math.min(height, from.y + Math.sin(angle) * this.STEP_SIZE * 1.8)),
      };
      const qNew = this.steer(from, target, this.STEP_SIZE);
      if (!this.isFree(qNew, obstacles) || !this.edgeFree(from, qNew, obstacles)) continue;

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: idx });
      steps.push({ type: 'edge', subtype: 'tree', x1: from.x, y1: from.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx });
        steps.push({ type: 'edge', subtype: 'tree', x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(EST);

// ---------------------------------------------------------------------------
// KPIECE1 – coarse projection grid exploration with boundary preference
// ---------------------------------------------------------------------------
class KPIECE1 extends SamplingAlgorithm {
  static id          = 'kpiece1';
  static displayName = 'KPIECE1';
  static description = 'Projection-based planner that biases exploration toward lightly explored boundary cells.';
  static paperUrl    = 'https://files.sucan.ro/ioan/files/pubs/wafr2008.pdf';
  static paperTitle  = 'Kinodynamic Motion Planning by Interior-Exterior Cell Exploration (I. A. Şucan and L. E. Kavraki, 2008)';
  static pseudocode  =
`Project states to coarse grid cells
Prioritize boundary and under-explored cells
Expand from nodes in selected cells`;

  static MAX_ITER = 2600;
  static STEP_SIZE = 22;
  static CELL_SIZE = 44;
  static GOAL_THRESHOLD = 20;

  static cellKey(p) {
    return `${Math.floor(p.x / this.CELL_SIZE)}:${Math.floor(p.y / this.CELL_SIZE)}`;
  }

  static isBoundaryCell(key, occupied) {
    const [cx, cy] = key.split(':').map(Number);
    const ns = [`${cx - 1}:${cy}`, `${cx + 1}:${cy}`, `${cx}:${cy - 1}`, `${cx}:${cy + 1}`];
    let missing = 0;
    for (const n of ns) if (!occupied.has(n)) missing += 1;
    return missing >= 2;
  }

  static weightedPick(entries) {
    const total = entries.reduce((s, e) => s + e.weight, 0);
    let r = Math.random() * total;
    for (let i = 0; i < entries.length; i++) {
      r -= entries[i].weight;
      if (r <= 0) return entries[i];
    }
    return entries[entries.length - 1];
  }

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1 }];
    const steps = [];
    const cellToNodes = new Map();
    const cellExpansions = new Map();

    const addNodeToCell = idx => {
      const k = this.cellKey(nodes[idx]);
      if (!cellToNodes.has(k)) cellToNodes.set(k, []);
      cellToNodes.get(k).push(idx);
      if (!cellExpansions.has(k)) cellExpansions.set(k, 0);
    };
    addNodeToCell(0);

    for (let i = 0; i < this.MAX_ITER; i++) {
      const occupied = new Set(cellToNodes.keys());
      const cellEntries = [...cellToNodes.entries()].map(([key, nodeIdxs]) => {
        const expandCount = cellExpansions.get(key) || 0;
        const boundaryBonus = this.isBoundaryCell(key, occupied) ? 2.5 : 1.0;
        const explorationBias = 1 / (1 + expandCount);
        return { key, nodeIdxs, weight: boundaryBonus * explorationBias };
      });

      const cell = this.weightedPick(cellEntries);
      const fromIdx = cell.nodeIdxs[(Math.random() * cell.nodeIdxs.length) | 0];
      const from = nodes[fromIdx];
      cellExpansions.set(cell.key, (cellExpansions.get(cell.key) || 0) + 1);

      const angle = Math.random() * Math.PI * 2;
      const target = {
        x: Math.max(0, Math.min(width, from.x + Math.cos(angle) * this.STEP_SIZE * 2)),
        y: Math.max(0, Math.min(height, from.y + Math.sin(angle) * this.STEP_SIZE * 2)),
      };
      const qNew = this.steer(from, target, this.STEP_SIZE);
      if (!this.isFree(qNew, obstacles) || !this.edgeFree(from, qNew, obstacles)) continue;

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: fromIdx });
      addNodeToCell(newIdx);
      steps.push({ type: 'edge', subtype: 'tree', x1: from.x, y1: from.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx });
        steps.push({ type: 'edge', subtype: 'tree', x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(KPIECE1);

// ---------------------------------------------------------------------------
// FMT* – one-shot sampled tree growth from an open frontier
// ---------------------------------------------------------------------------
class FMTStar extends SamplingAlgorithm {
  static id          = 'fmt';
  static displayName = 'FMT*';
  static description = 'Fast Marching Tree over random geometric graph samples.';
  static paperUrl    = 'https://arxiv.org/abs/1306.3532';
  static paperTitle  = 'Fast Marching Tree: A Fast Marching Sampling-Based Method For Optimal Motion Planning in Many Dimensions (Janson et al., 2013)';
  static pseudocode  =
`Sample free states and build an implicit neighborhood graph
Grow a tree from the start over the open frontier
Attach each unvisited sample through the cheapest open neighbor`;

  static N_SAMPLES = 240;
  static NEIGHBOR_RADIUS = 85;

  static run({ width, height, obstacles, start, end }) {
    const steps = [];
    const nodes = [{ x: start.x, y: start.y }, { x: end.x, y: end.y }];
    for (let i = 0; i < this.N_SAMPLES; i++) nodes.push(this.randomFreePoint(width, height, obstacles));

    const neighbors = Array.from({ length: nodes.length }, () => []);
    for (let i = 0; i < nodes.length; i++) {
      for (let j = i + 1; j < nodes.length; j++) {
        if (this.dist(nodes[i], nodes[j]) <= this.NEIGHBOR_RADIUS) {
          neighbors[i].push(j);
          neighbors[j].push(i);
        }
      }
    }

    const cost = new Array(nodes.length).fill(Infinity);
    const parent = new Array(nodes.length).fill(-1);
    const inOpen = new Array(nodes.length).fill(false);
    const unvisited = new Set(Array.from({ length: nodes.length - 1 }, (_, k) => k + 1));
    const open = new Set([0]);
    inOpen[0] = true;
    cost[0] = 0;

    while (open.size > 0) {
      let x = -1;
      let best = Infinity;
      for (const idx of open) {
        if (cost[idx] < best) { best = cost[idx]; x = idx; }
      }
      if (x === 1) break;

      const newlyOpened = [];
      for (const y of neighbors[x]) {
        if (!unvisited.has(y)) continue;

        let minParent = -1;
        let minCost = Infinity;
        for (const n of neighbors[y]) {
          if (!inOpen[n]) continue;
          if (!this.edgeFree(nodes[n], nodes[y], obstacles)) continue;
          const c = cost[n] + this.dist(nodes[n], nodes[y]);
          if (c < minCost) { minCost = c; minParent = n; }
        }

        if (minParent !== -1) {
          parent[y] = minParent;
          cost[y] = minCost;
          newlyOpened.push(y);
          unvisited.delete(y);
          steps.push({ type: 'edge', subtype: 'tree',
            x1: nodes[minParent].x, y1: nodes[minParent].y, x2: nodes[y].x, y2: nodes[y].y });
        }
      }

      open.delete(x);
      inOpen[x] = false;
      for (const y of newlyOpened) {
        open.add(y);
        inOpen[y] = true;
      }
    }

    if (parent[1] === -1) return { steps, path: null };
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
Algorithm.register(FMTStar);

// ---------------------------------------------------------------------------
// BIT* – batch-informed random geometric graph search (static-site variant)
// ---------------------------------------------------------------------------
class BITStar extends SamplingAlgorithm {
  static id          = 'bitstar';
  static displayName = 'BIT*';
  static description = 'Batch-informed planning: iteratively grows a sampled graph and searches it with heuristic guidance.';
  static paperUrl    = 'https://arxiv.org/abs/1707.01888';
  static paperTitle  = 'Batch Informed Trees (BIT*): Informed asymptotically optimal anytime search (Gammell et al., 2020)';
  static pseudocode  =
`Repeat over batches:
  Add new random (or informed) samples
  Build local random-geometric graph connections
  Run heuristic search (A*) on the implicit graph
Keep the best solution found so far`;

  static MAX_BATCHES = 6;
  static BATCH_SIZE = 90;
  static CONNECT_RADIUS = 72;

  static reconstructPath(parent, nodes, goalIdx) {
    if (parent[goalIdx] === -1) return null;
    const path = [];
    let cur = goalIdx;
    while (cur !== -1) {
      path.unshift({ x: nodes[cur].x, y: nodes[cur].y });
      cur = parent[cur];
    }
    return path;
  }

  static runAStar(nodes, startIdx, goalIdx, adj, obstacles, steps) {
    const n = nodes.length;
    const g = new Array(n).fill(Infinity);
    const f = new Array(n).fill(Infinity);
    const parent = new Array(n).fill(-1);
    const open = new Set([startIdx]);
    const closed = new Set();

    g[startIdx] = 0;
    f[startIdx] = this.dist(nodes[startIdx], nodes[goalIdx]);

    while (open.size > 0) {
      let cur = -1;
      let bestF = Infinity;
      for (const idx of open) {
        if (f[idx] < bestF) { bestF = f[idx]; cur = idx; }
      }
      if (cur === goalIdx) return { found: true, g, parent };

      open.delete(cur);
      closed.add(cur);

      for (const nb of adj[cur]) {
        if (closed.has(nb)) continue;
        if (!this.edgeFree(nodes[cur], nodes[nb], obstacles)) continue;

        const tentative = g[cur] + this.dist(nodes[cur], nodes[nb]);
        if (tentative + 1e-6 < g[nb]) {
          parent[nb] = cur;
          g[nb] = tentative;
          f[nb] = tentative + this.dist(nodes[nb], nodes[goalIdx]);
          open.add(nb);
          steps.push({ type: 'edge', subtype: 'roadmap',
            x1: nodes[cur].x, y1: nodes[cur].y, x2: nodes[nb].x, y2: nodes[nb].y });
        }
      }
    }
    return { found: false, g, parent };
  }

  static run({ width, height, obstacles, start, end }) {
    const steps = [];
    const nodes = [{ x: start.x, y: start.y }, { x: end.x, y: end.y }];
    let bestPath = null;
    let bestCost = Infinity;

    for (let batch = 0; batch < this.MAX_BATCHES; batch++) {
      for (let i = 0; i < this.BATCH_SIZE; i++) {
        const sample = bestPath
          ? this.informedSample(start, end, bestCost, width, height, obstacles)
          : this.randomFreePoint(width, height, obstacles);
        nodes.push(sample);
      }

      const adj = Array.from({ length: nodes.length }, () => []);
      for (let i = 0; i < nodes.length; i++) {
        for (let j = i + 1; j < nodes.length; j++) {
          if (this.dist(nodes[i], nodes[j]) <= this.CONNECT_RADIUS) {
            adj[i].push(j);
            adj[j].push(i);
          }
        }
      }

      const res = this.runAStar(nodes, 0, 1, adj, obstacles, steps);
      if (!res.found) continue;
      const path = this.reconstructPath(res.parent, nodes, 1);
      if (!path) continue;
      const cost = res.g[1];
      if (cost < bestCost) {
        bestCost = cost;
        bestPath = path;
      }
    }

    if (!bestPath) return { steps, path: null };
    steps.push({ type: 'path', points: bestPath });
    return { steps, path: bestPath };
  }
}
Algorithm.register(BITStar);
