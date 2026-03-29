'use strict';

/* ============================================================
   js/sampling-algorithms.js
   Each algorithm is defined as two classes:
     [Name]Info  extends SamplingAlgorithmInfo  — metadata only
     [Name]Impl  extends SamplingAlgorithmImpl  — run() only
   Paired via Algorithm.register(Info, Impl).

   run(env) -> { steps: Step[], path: Point[]|null }
     env  = { width, height, obstacles, start, end }
     Step = { type:'edge', subtype:'tree'|'roadmap'|'collision', x1,y1,x2,y2 }
          | { type:'path', points:[{x,y}...] }
   ============================================================ */

// ============================================================
// RRT – Rapidly-exploring Random Tree
// ============================================================
class RRTInfo extends SamplingAlgorithmInfo {
  static id          = 'rrt';
  static displayName = 'RRT';
  static description = 'Rapidly-exploring Random Tree. Grows a tree by sampling the space at random and steering toward samples. Probabilistically complete but not optimal.';
  static paperUrl    = 'http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf';
  static paperTitle  = 'Rapidly-Exploring Random Trees: A New Tool for Path Planning (LaValle, 1998)';
  static pseudocode  =
`T <- tree rooted at start

Repeat until goal reached or MAX_ITER exceeded:
  q_rand <- random point in space  (goal with prob GOAL_BIAS)
  q_near <- nearest node in T to q_rand
  q_new  <- steer(q_near -> q_rand, step = STEP_SIZE)

  If q_new is in free space:
    If edge(q_near, q_new) is obstacle-free:
      Add q_new to T,  parent(q_new) <- q_near
      If dist(q_new, goal) < GOAL_THRESHOLD and edge clear:
        Connect q_new -> goal
        Retrace parents: goal -> ... -> start
        Return path
    Else:
      Record collision attempt (visualised in red)

Return  failure  (no path found in MAX_ITER iterations)`;

  static year               = 1998;
  static timeComplexity     = 'O(n log n) expected';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'No — finds a path, not the shortest one';
  static complete           = 'Probabilistically complete';
  static predecessor        = null;
  static notableImprovement = 'First practical randomized tree planner for high-dimensional spaces';
  static successor          = 'RRT*, RRTConnect, InformedRRT*';
  static goodFor            = ['Single-query planning', 'High-dimensional spaces', 'Fast initial solutions'];
  static badFor             = ['Optimal paths', 'Narrow passages (low probability of sampling through)'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class RRTImpl extends SamplingAlgorithmImpl {
  static id = 'rrt';

  static MAX_ITER       = 3000;
  static STEP_SIZE      = 25;
  static GOAL_THRESHOLD = 20;
  static GOAL_BIAS      = 0.05;

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1 }];
    const steps = [];

    for (let i = 0; i < this.MAX_ITER; i++) {
      const qRand = Math.random() < this.GOAL_BIAS
        ? { x: end.x, y: end.y }
        : this.randomPoint(width, height);

      const nearIdx = this.nearestIdx(nodes, qRand);
      const qNear   = nodes[nearIdx];
      const qNew    = this.steer(qNear, qRand, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) continue;
      if (!this.edgeFree(qNear, qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: qNear.x, y1: qNear.y, x2: qNew.x, y2: qNew.y });
        continue;
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: qNear.x, y1: qNear.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx });
        steps.push({ type: 'edge', subtype: 'tree',
          x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(RRTInfo, RRTImpl);

// ============================================================
// RRT* – Asymptotically Optimal RRT
// ============================================================
class RRTStarInfo extends SamplingAlgorithmInfo {
  static id          = 'rrt-star';
  static displayName = 'RRT*';
  static description = 'Asymptotically optimal extension of RRT. Rewires the tree to minimise path cost each iteration, converging toward the global optimum as samples increase.';
  static paperUrl    = 'https://doi.org/10.1177/0278364911406761';
  static paperTitle  = 'Sampling-based Algorithms for Optimal Motion Planning (Karaman & Frazzoli, 2011)';
  static pseudocode  =
`T <- tree rooted at start,  cost[start] <- 0

For i = 1 to MAX_ITER:
  q_rand <- random point in free space
  q_near <- nearest node in T to q_rand
  q_new  <- steer(q_near -> q_rand, step = STEP_SIZE)

  If q_new is free and edge is clear:
    Q_near <- nodes in T within NEIGHBOR_RADIUS with clear edge to q_new

    // Choose minimum-cost parent
    q_min  <- argmin_{q in Q_near}  cost[q] + dist(q, q_new)
    Add q_new with parent q_min;  cost[q_new] <- cost[q_min] + dist(q_min, q_new)

    // Rewire neighbourhood
    For each q in Q_near:
      If cost[q_new] + dist(q_new, q) < cost[q]:
        Re-parent q to q_new;  update cost[q]

    Track best collision-free connection to goal

Return best path found  (or failure if goal never reached)`;

  static year               = 2011;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'Asymptotically optimal';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'RRT';
  static notableImprovement = 'Near-neighbour rewiring guarantees asymptotic optimality';
  static successor          = 'InformedRRT*, BIT*';
  static goodFor            = ['Optimal paths', 'Anytime planning', 'Continuous environments'];
  static badFor             = ['Real-time planning (slower convergence than RRT)', 'Memory-constrained settings'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class RRTStarImpl extends SamplingAlgorithmImpl {
  static id = 'rrt-star';

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
      const qRand   = this.randomPoint(width, height);
      const nearIdx = this.nearestIdx(nodes, qRand);
      const qNew    = this.steer(nodes[nearIdx], qRand, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) continue;
      if (!this.edgeFree(nodes[nearIdx], qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: nodes[nearIdx].x, y1: nodes[nearIdx].y, x2: qNew.x, y2: qNew.y });
        continue;
      }

      // Collect obstacle-free neighbours within NEIGHBOR_RADIUS
      const nbrIdxs = [];
      for (let j = 0; j < nodes.length; j++) {
        if (this.dist(nodes[j], qNew) <= this.NEIGHBOR_RADIUS &&
            this.edgeFree(nodes[j], qNew, obstacles)) {
          nbrIdxs.push(j);
        }
      }

      // Choose minimum-cost parent (fall back to nearest if no neighbours)
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
        }
      }

      // Track best goal connection
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
    const path = this.tracePath(nodes, nodes.length - 1);
    steps.push({ type: 'path', points: path });
    return { steps, path };
  }
}
Algorithm.register(RRTStarInfo, RRTStarImpl);

// ============================================================
// PRM – Probabilistic Roadmap Method
// ============================================================
class PRMInfo extends SamplingAlgorithmInfo {
  static id          = 'prm';
  static displayName = 'PRM';
  static description = 'Probabilistic Roadmap Method. Samples free space to build a reusable roadmap graph, then queries it with BFS to find a path between start and goal.';
  static paperUrl    = 'https://doi.org/10.1109/70.508439';
  static paperTitle  = 'Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces (Kavraki et al., 1996)';
  static pseudocode  =
`-- Learning Phase --
For i = 1 to N_SAMPLES:
  q <- random free configuration
  Add q to roadmap R
  For each node n in R within CONNECT_RADIUS of q:
    If edge(q, n) is obstacle-free:
      Add edge (q <-> n) to R
    Else:
      Record collision attempt (visualised in red)

-- Query Phase --
Connect start and goal into R with the same rule
Run BFS on R from start to goal
If reachable: trace parents and return path
Else: return failure`;

  static year               = 1996;
  static timeComplexity     = 'O(n² log n)';
  static spaceComplexity    = 'O(n²)';
  static optimal            = 'No — roadmap quality depends on sample density';
  static complete           = 'Probabilistically complete';
  static predecessor        = null;
  static notableImprovement = 'Reusable roadmap amortises construction cost across multiple queries';
  static successor          = 'FMT*, BIT*';
  static goodFor            = ['Multi-query planning', 'Static environments', 'Well-connected spaces'];
  static badFor             = ['Dynamic environments', 'Narrow passages', 'Single-query tasks (RRT is faster)'];
  static samplingStrategy   = 'roadmap';
  static queryType          = 'multi';
}

class PRMImpl extends SamplingAlgorithmImpl {
  static id = 'prm';

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
        if (this.dist(nodes[i], nodes[j]) > this.CONNECT_RADIUS) continue;
        if (this.edgeFree(nodes[i], nodes[j], obstacles)) {
          adj[i].push(j);
          adj[j].push(i);
          steps.push({ type: 'edge', subtype: 'roadmap',
            x1: nodes[i].x, y1: nodes[i].y, x2: nodes[j].x, y2: nodes[j].y });
        } else {
          steps.push({ type: 'edge', subtype: 'collision',
            x1: nodes[i].x, y1: nodes[i].y, x2: nodes[j].x, y2: nodes[j].y });
        }
      }
    }

    // BFS from start (0) to goal (1)
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
Algorithm.register(PRMInfo, PRMImpl);

// ============================================================
// RRTConnect – Bidirectional RRT
// ============================================================
class RRTConnectInfo extends SamplingAlgorithmInfo {
  static id          = 'rrt-connect';
  static displayName = 'RRTConnect';
  static description = 'Bidirectional RRT. Grows two trees simultaneously – one from start, one from goal – and greedily tries to connect them each iteration.';
  static paperUrl    = 'http://www.kuffner.org/james/papers/kuffner_icra2000.pdf';
  static paperTitle  = 'RRT-Connect: An Efficient Approach to Single-Query Path Planning (Kuffner & LaValle, 2000)';
  static pseudocode  =
`T_a <- tree rooted at start
T_b <- tree rooted at goal

Repeat until MAX_ITER:
  q_rand <- random sample
  q_new  <- EXTEND(T_a, q_rand)   // one step toward q_rand
  If q_new was added:
    CONNECT(T_b, q_new)            // greedily step T_b toward q_new
    If T_b reached q_new:
      Merge paths T_a and T_b
      Return path(start -> ... -> goal)
  Swap T_a and T_b

Return failure`;

  static year               = 2000;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'No — finds a path quickly but not guaranteed optimal';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'RRT';
  static notableImprovement = 'Bidirectional growth drastically reduces time to first solution';
  static successor          = null;
  static goodFor            = ['Fastest initial solutions', 'Single-query planning', 'Open spaces'];
  static badFor             = ['Optimal paths', 'Environments requiring careful cost management'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class RRTConnectImpl extends SamplingAlgorithmImpl {
  static id = 'rrt-connect';

  static MAX_ITER  = 2200;
  static STEP_SIZE = 25;

  /** One step of T toward target; returns new node index or -1 on failure. */
  static _extend(nodes, target, obstacles, steps) {
    const nearIdx = this.nearestIdx(nodes, target);
    const near    = nodes[nearIdx];
    const qNew    = this.steer(near, target, this.STEP_SIZE);

    if (!this.isFree(qNew, obstacles)) return -1;
    if (!this.edgeFree(near, qNew, obstacles)) {
      steps.push({ type: 'edge', subtype: 'collision',
        x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
      return -1;
    }

    const newIdx = nodes.length;
    nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx });
    steps.push({ type: 'edge', subtype: 'tree',
      x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
    return newIdx;
  }

  /** Greedily step T toward target until it reaches or is blocked. */
  static _connect(nodes, target, obstacles, steps) {
    let lastIdx = -1;
    while (true) {
      const nearIdx = this.nearestIdx(nodes, target);
      const near    = nodes[nearIdx];
      const d       = this.dist(near, target);
      const qNew    = this.steer(near, target, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) break;
      if (!this.edgeFree(near, qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
        break;
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
      lastIdx = newIdx;

      if (d <= this.STEP_SIZE) return { connected: true, idx: newIdx };
    }
    return { connected: false, idx: lastIdx };
  }

  /** Traces from a node to its tree root, returning nodes leaf-first. */
  static _trace(nodes, idx) {
    const out = [];
    let cur = idx;
    while (cur !== -1) { out.push({ x: nodes[cur].x, y: nodes[cur].y }); cur = nodes[cur].parentIdx; }
    return out;
  }

  static run({ width, height, obstacles, start, end }) {
    const steps = [];

    let treeA      = [{ x: start.x, y: start.y, parentIdx: -1 }];
    let treeB      = [{ x: end.x,   y: end.y,   parentIdx: -1 }];
    let aFromStart = true;

    for (let i = 0; i < this.MAX_ITER; i++) {
      const qRand   = this.randomPoint(width, height);
      const newAIdx = this._extend(treeA, qRand, obstacles, steps);

      if (newAIdx !== -1) {
        const qNew       = treeA[newAIdx];
        const connectRes = this._connect(treeB, qNew, obstacles, steps);

        if (connectRes.connected) {
          // _trace returns leaf-first; .reverse() gives root-first
          const aBranch = this._trace(treeA, newAIdx).reverse(); // root_A -> newA
          const bBranch = this._trace(treeB, connectRes.idx);    // connectB -> root_B

          const path = aFromStart
            ? aBranch.concat(bBranch)                        // start->newA + connectB->goal
            : bBranch.reverse().concat(aBranch.reverse());   // start->connectB + newA->goal
          steps.push({ type: 'path', points: path });
          return { steps, path };
        }
      }

      // Swap trees each iteration (classic RRT-Connect strategy)
      const tmp = treeA;
      treeA = treeB;
      treeB = tmp;
      aFromStart = !aFromStart;
    }

    return { steps, path: null };
  }
}
Algorithm.register(RRTConnectInfo, RRTConnectImpl);

// ============================================================
// InformedRRT* – informed subset sampling
// ============================================================
class InformedRRTStarInfo extends SamplingAlgorithmInfo {
  static id          = 'informed-rrt-star';
  static displayName = 'InformedRRT*';
  static description = 'RRT* variant that, once an initial solution is found, restricts sampling to an ellipsoidal subset guaranteed to improve the current best cost.';
  static paperUrl    = 'https://arxiv.org/abs/1404.2334';
  static paperTitle  = 'Informed RRT*: Optimal Sampling-based Path Planning via Direct Sampling of an Admissible Ellipsoidal Heuristic (Gammell et al., 2014)';
  static pseudocode  =
`Run RRT* until first solution found with cost c_best

Then, sample only from the prolate hyperspheroid:
  ellipse centred on midpoint(start, goal)
  semi-major axis a = c_best / 2
  semi-minor axis b = sqrt(c_best^2 - c_min^2) / 2
  (c_min = straight-line distance start->goal)

Continue rewiring within that informed subset
Each new solution tightens the ellipse`;

  static year               = 2014;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'Asymptotically optimal';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'RRT*';
  static notableImprovement = 'Ellipsoidal informed sampling shrinks search space after first solution, accelerating convergence';
  static successor          = 'BIT*';
  static goodFor            = ['Fast convergence to optimum', 'When initial solution is found quickly', 'Tight spaces requiring precise paths'];
  static badFor             = ['Environments where an initial solution is very hard to find'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class InformedRRTStarImpl extends SamplingAlgorithmImpl {
  static id = 'informed-rrt-star';

  static MAX_ITER        = 2500;
  static STEP_SIZE       = 24;
  static GOAL_THRESHOLD  = 18;
  static NEIGHBOR_RADIUS = 60;
  static GOAL_BIAS       = 0.07;

  static run({ width, height, obstacles, start, end }) {
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1, cost: 0 }];
    const steps = [];
    let bestGoalParentIdx = -1;
    let bestGoalCost      = Infinity;

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
      const qNew    = this.steer(nodes[nearIdx], qRand, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) continue;
      if (!this.edgeFree(nodes[nearIdx], qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: nodes[nearIdx].x, y1: nodes[nearIdx].y, x2: qNew.x, y2: qNew.y });
        continue;
      }

      // Neighbour collection
      const nbrIdxs = [];
      for (let j = 0; j < nodes.length; j++) {
        if (this.dist(nodes[j], qNew) <= this.NEIGHBOR_RADIUS &&
            this.edgeFree(nodes[j], qNew, obstacles)) {
          nbrIdxs.push(j);
        }
      }
      if (nbrIdxs.length === 0) nbrIdxs.push(nearIdx);

      // Best parent
      let minParent = nbrIdxs[0];
      let minCost   = nodes[minParent].cost + this.dist(nodes[minParent], qNew);
      for (const j of nbrIdxs) {
        const c = nodes[j].cost + this.dist(nodes[j], qNew);
        if (c < minCost) { minCost = c; minParent = j; }
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: minParent, cost: minCost });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: nodes[minParent].x, y1: nodes[minParent].y, x2: qNew.x, y2: qNew.y });

      // Rewire
      for (const j of nbrIdxs) {
        const rewireCost = nodes[newIdx].cost + this.dist(nodes[newIdx], nodes[j]);
        if (rewireCost + 1e-6 < nodes[j].cost &&
            this.edgeFree(nodes[newIdx], nodes[j], obstacles)) {
          nodes[j].parentIdx = newIdx;
          nodes[j].cost      = rewireCost;
        }
      }

      // Goal check
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
    const path = this.tracePath(nodes, nodes.length - 1);
    steps.push({ type: 'path', points: path });
    return { steps, path };
  }
}
Algorithm.register(InformedRRTStarInfo, InformedRRTStarImpl);

// ============================================================
// TRRT – Transition-based RRT
// ============================================================
class TRRTInfo extends SamplingAlgorithmInfo {
  static id          = 'trrt';
  static displayName = 'TRRT';
  static description = 'Transition-based RRT. Accepts moves toward higher-cost (lower-clearance) states with probability exp(-delta/T), using a temperature schedule to balance exploration and obstacle avoidance.';
  static paperUrl    = 'https://doi.org/10.1109/IROS.2008.4650993';
  static paperTitle  = 'Transition-based RRT for Path Planning in Continuous Cost Spaces (Jaillet, Cortes, Simeon, 2008)';
  static pseudocode  =
`cost(p) = 1 / (clearance(p) + 1)   // higher near obstacles
T <- T_INIT                          // temperature

Grow an RRT with probabilistic acceptance:
  q_new  <- steer from nearest(q_rand)
  delta  <- cost(q_new) - cost(q_near)
  accept <- delta <= 0  or  rand() < exp(-delta / T)

  If accept:
    Add q_new to tree;  T <- T * 0.995  (cool)
  Else:
    Reject;  T <- T * 1.02  (heat)

Return path if goal reached, else failure`;

  static year               = 2008;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'No — cost-aware but not asymptotically optimal';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'RRT';
  static notableImprovement = 'Temperature schedule biases tree away from high-cost (low-clearance) regions';
  static successor          = null;
  static goodFor            = ['Maximising clearance from obstacles', 'Continuous cost-space planning', 'Cost-map aware environments'];
  static badFor             = ['Optimal path length', 'Environments without meaningful cost gradients'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class TRRTImpl extends SamplingAlgorithmImpl {
  static id = 'trrt';

  static MAX_ITER       = 2600;
  static STEP_SIZE      = 22;
  static GOAL_THRESHOLD = 20;
  static GOAL_BIAS      = 0.05;
  static T_INIT         = 0.12;

  /** State cost: 1/(clearance+1), so low clearance = high cost. */
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
    const nodes = [{ x: start.x, y: start.y, parentIdx: -1,
                     cost: this.stateCost(start, obstacles) }];
    const steps = [];
    let temperature = this.T_INIT;

    for (let i = 0; i < this.MAX_ITER; i++) {
      const qRand   = Math.random() < this.GOAL_BIAS
        ? { x: end.x, y: end.y }
        : this.randomFreePoint(width, height, obstacles);
      const nearIdx = this.nearestIdx(nodes, qRand);
      const near    = nodes[nearIdx];
      const qNew    = this.steer(near, qRand, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) continue;
      if (!this.edgeFree(near, qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });
        continue;
      }

      const cNew   = this.stateCost(qNew, obstacles);
      const delta  = cNew - near.cost;
      const accept = delta <= 0 || Math.random() < Math.exp(-delta / Math.max(temperature, 1e-5));
      if (!accept) { temperature *= 1.02; continue; }
      temperature *= 0.995;

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: nearIdx, cost: cNew });
      steps.push({ type: 'edge', subtype: 'tree',
        x1: near.x, y1: near.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx, cost: 0 });
        steps.push({ type: 'edge', subtype: 'tree',
          x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(TRRTInfo, TRRTImpl);

// ============================================================
// EST – Expansive Space Trees
// ============================================================
class ESTInfo extends SamplingAlgorithmInfo {
  static id          = 'est';
  static displayName = 'EST';
  static description = 'Expansive Space Trees. Biases expansion toward sparsely explored regions of the tree, improving coverage in high-dimensional or narrow-passage environments.';
  static paperUrl    = 'https://doi.org/10.1142/S0218195999000285';
  static paperTitle  = 'Path Planning in Expansive Configuration Spaces (Hsu, Latombe, Motwani, 1999)';
  static pseudocode  =
`T <- {start}
density[v] <- number of T-neighbours within DENSITY_RADIUS of v

Repeat:
  // Weight nodes inversely by local density
  Pick node v from T with probability proportional to 1/(1+density[v])
  Sample random direction from v
  q_new <- steer(v, direction, STEP_SIZE)

  If q_new is free and edge is clear:
    Add q_new to T;  update density of nearby nodes
    If goal reached: return path

Return failure`;

  static year               = 1999;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'No';
  static complete           = 'Probabilistically complete';
  static predecessor        = null;
  static notableImprovement = 'Density-inversely-weighted expansion improves coverage in cluttered environments';
  static successor          = 'KPIECE1';
  static goodFor            = ['High-dimensional spaces', 'Narrow passages', 'Expansive environments'];
  static badFor             = ['Optimal paths', 'Real-time applications'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class ESTImpl extends SamplingAlgorithmImpl {
  static id = 'est';

  static MAX_ITER       = 2600;
  static STEP_SIZE      = 22;
  static DENSITY_RADIUS = 55;
  static GOAL_THRESHOLD = 20;

  /** Weighted random index selection from an array of weights. */
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
    const nodes   = [{ x: start.x, y: start.y, parentIdx: -1 }];
    const steps   = [];
    const density = [0]; // incremental density count per node

    for (let i = 0; i < this.MAX_ITER; i++) {
      // Choose node inversely weighted by local density
      const weights = density.map(d => 1 / (1 + d));
      const idx     = this.weightedIndex(weights);
      const from    = nodes[idx];

      // Random motion from selected node
      const angle  = Math.random() * Math.PI * 2;
      const target = {
        x: Math.max(0, Math.min(width,  from.x + Math.cos(angle) * this.STEP_SIZE * 1.8)),
        y: Math.max(0, Math.min(height, from.y + Math.sin(angle) * this.STEP_SIZE * 1.8)),
      };
      const qNew = this.steer(from, target, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) continue;
      if (!this.edgeFree(from, qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: from.x, y1: from.y, x2: qNew.x, y2: qNew.y });
        continue;
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: idx });
      density.push(0);
      steps.push({ type: 'edge', subtype: 'tree',
        x1: from.x, y1: from.y, x2: qNew.x, y2: qNew.y });

      // Update density incrementally for all nodes near qNew
      for (let j = 0; j < newIdx; j++) {
        if (this.dist(nodes[j], qNew) < this.DENSITY_RADIUS) {
          density[j]++;
          density[newIdx]++;
        }
      }

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx });
        steps.push({ type: 'edge', subtype: 'tree',
          x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(ESTInfo, ESTImpl);

// ============================================================
// KPIECE1 – Kinodynamic Planning by Interior-Exterior Cell Exploration
// ============================================================
class KPIECE1Info extends SamplingAlgorithmInfo {
  static id          = 'kpiece1';
  static displayName = 'KPIECE1';
  static description = 'Projects states onto a coarse grid and biases expansion toward boundary and under-explored cells, encouraging the tree to push into unvisited space.';
  static paperUrl    = 'https://files.sucan.ro/ioan/files/pubs/wafr2008.pdf';
  static paperTitle  = 'Kinodynamic Motion Planning by Interior-Exterior Cell Exploration (Şucan & Kavraki, 2008)';
  static pseudocode  =
`Project each tree node into a coarse grid cell
Assign each cell a weight:
  weight = (boundary bonus) * (1 / (1 + expansion_count))

Repeat:
  cell <- weighted-random cell selection
  from <- random node in cell
  q_new <- steer(from, random direction, STEP_SIZE)

  If free and clear:
    Add q_new;  update cell counts
    If goal reached: return path

Return failure`;

  static year               = 2009;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'No';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'EST';
  static notableImprovement = 'Cell-projection biases expansion toward unexplored boundary regions, outperforming EST in cluttered spaces';
  static successor          = null;
  static goodFor            = ['Kinodynamic planning', 'Boundary exploration', 'Under-explored region bias'];
  static badFor             = ['Optimal paths', 'Low-dimensional spaces where EST or RRT suffice'];
  static samplingStrategy   = 'tree';
  static queryType          = 'single';
}

class KPIECE1Impl extends SamplingAlgorithmImpl {
  static id = 'kpiece1';

  static MAX_ITER       = 2600;
  static STEP_SIZE      = 22;
  static CELL_SIZE      = 44;
  static GOAL_THRESHOLD = 20;

  static cellKey(p) {
    return `${Math.floor(p.x / this.CELL_SIZE)}:${Math.floor(p.y / this.CELL_SIZE)}`;
  }

  static isBoundaryCell(key, occupied) {
    const [cx, cy] = key.split(':').map(Number);
    const ns = [`${cx-1}:${cy}`, `${cx+1}:${cy}`, `${cx}:${cy-1}`, `${cx}:${cy+1}`];
    return ns.filter(n => !occupied.has(n)).length >= 2;
  }

  static weightedPick(entries) {
    const total = entries.reduce((s, e) => s + e.weight, 0);
    let r = Math.random() * total;
    for (const e of entries) { r -= e.weight; if (r <= 0) return e; }
    return entries[entries.length - 1];
  }

  static run({ width, height, obstacles, start, end }) {
    const nodes          = [{ x: start.x, y: start.y, parentIdx: -1 }];
    const steps          = [];
    const cellToNodes    = new Map();
    const cellExpansions = new Map();

    const addToCell = idx => {
      const k = this.cellKey(nodes[idx]);
      if (!cellToNodes.has(k)) { cellToNodes.set(k, []); cellExpansions.set(k, 0); }
      cellToNodes.get(k).push(idx);
    };
    addToCell(0);

    for (let i = 0; i < this.MAX_ITER; i++) {
      const occupied    = new Set(cellToNodes.keys());
      const cellEntries = [...cellToNodes.entries()].map(([key, nodeIdxs]) => ({
        key, nodeIdxs,
        weight: (this.isBoundaryCell(key, occupied) ? 2.5 : 1.0)
               / (1 + (cellExpansions.get(key) || 0)),
      }));

      const cell    = this.weightedPick(cellEntries);
      const fromIdx = cell.nodeIdxs[(Math.random() * cell.nodeIdxs.length) | 0];
      const from    = nodes[fromIdx];
      cellExpansions.set(cell.key, (cellExpansions.get(cell.key) || 0) + 1);

      const angle  = Math.random() * Math.PI * 2;
      const target = {
        x: Math.max(0, Math.min(width,  from.x + Math.cos(angle) * this.STEP_SIZE * 2)),
        y: Math.max(0, Math.min(height, from.y + Math.sin(angle) * this.STEP_SIZE * 2)),
      };
      const qNew = this.steer(from, target, this.STEP_SIZE);

      if (!this.isFree(qNew, obstacles)) continue;
      if (!this.edgeFree(from, qNew, obstacles)) {
        steps.push({ type: 'edge', subtype: 'collision',
          x1: from.x, y1: from.y, x2: qNew.x, y2: qNew.y });
        continue;
      }

      const newIdx = nodes.length;
      nodes.push({ x: qNew.x, y: qNew.y, parentIdx: fromIdx });
      addToCell(newIdx);
      steps.push({ type: 'edge', subtype: 'tree',
        x1: from.x, y1: from.y, x2: qNew.x, y2: qNew.y });

      if (this.dist(qNew, end) < this.GOAL_THRESHOLD && this.edgeFree(qNew, end, obstacles)) {
        nodes.push({ x: end.x, y: end.y, parentIdx: newIdx });
        steps.push({ type: 'edge', subtype: 'tree',
          x1: qNew.x, y1: qNew.y, x2: end.x, y2: end.y });
        const path = this.tracePath(nodes, nodes.length - 1);
        steps.push({ type: 'path', points: path });
        return { steps, path };
      }
    }
    return { steps, path: null };
  }
}
Algorithm.register(KPIECE1Info, KPIECE1Impl);

// ============================================================
// FMT* – Fast Marching Tree
// ============================================================
class FMTStarInfo extends SamplingAlgorithmInfo {
  static id          = 'fmt';
  static displayName = 'FMT*';
  static description = 'Fast Marching Tree. Pre-samples the free space, builds a lazy geometric graph, then grows a tree by expanding the open frontier in order of cost — a single-pass, near-optimal planner.';
  static paperUrl    = 'https://arxiv.org/abs/1306.3532';
  static paperTitle  = 'Fast Marching Tree: A Fast Marching Sampling-Based Method For Optimal Motion Planning in Many Dimensions (Janson et al., 2013)';
  static pseudocode  =
`Sample N_SAMPLES free states; add start and goal
Build implicit geometric graph: edges within NEIGHBOR_RADIUS

V_open <- {start},  V_unvisited <- all others
cost[start] <- 0

While V_open is not empty:
  x <- node in V_open with lowest cost
  If x is goal: return path

  For each y in V_unvisited ∩ neighbors(x):
    Find z in V_open ∩ neighbors(y) with min  cost[z] + dist(z, y)
    If z exists and edge(z, y) is clear:
      parent[y] <- z;  cost[y] <- cost[z] + dist(z, y)
      Move y from V_unvisited -> V_open

  Move x from V_open -> V_closed

Return failure`;

  static year               = 2015;
  static timeComplexity     = 'O(n log n)';
  static spaceComplexity    = 'O(n²)';
  static optimal            = 'Asymptotically optimal';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'PRM, RRT*';
  static notableImprovement = 'Fast marching wave expansion achieves near-optimal paths in a single forward pass';
  static successor          = 'BIT*';
  static goodFor            = ['Batch planning', 'Near-optimal solutions with fixed sample set', 'Static environments'];
  static badFor             = ['Dynamic environments', 'Real-time replanning', 'Very high dimensions (O(n²) neighbours)'];
  static samplingStrategy   = 'batch';
  static queryType          = 'single';
}

class FMTStarImpl extends SamplingAlgorithmImpl {
  static id = 'fmt';

  static N_SAMPLES       = 240;
  static NEIGHBOR_RADIUS = 85;

  static run({ width, height, obstacles, start, end }) {
    const steps = [];
    const nodes = [{ x: start.x, y: start.y }, { x: end.x, y: end.y }];
    for (let i = 0; i < this.N_SAMPLES; i++) {
      nodes.push(this.randomFreePoint(width, height, obstacles));
    }

    // Build geometric neighbor graph (no collision check yet – lazy)
    const neighbors = Array.from({ length: nodes.length }, () => []);
    for (let i = 0; i < nodes.length; i++) {
      for (let j = i + 1; j < nodes.length; j++) {
        if (this.dist(nodes[i], nodes[j]) <= this.NEIGHBOR_RADIUS) {
          neighbors[i].push(j);
          neighbors[j].push(i);
        }
      }
    }

    const cost      = new Array(nodes.length).fill(Infinity);
    const parent    = new Array(nodes.length).fill(-1);
    const inOpen    = new Array(nodes.length).fill(false);
    const unvisited = new Set(Array.from({ length: nodes.length - 1 }, (_, k) => k + 1));
    const open      = new Set([0]);
    inOpen[0] = true;
    cost[0]   = 0;

    while (open.size > 0) {
      // Pick lowest-cost open node
      let x = -1, best = Infinity;
      for (const idx of open) { if (cost[idx] < best) { best = cost[idx]; x = idx; } }
      if (x === 1) break; // goal reached

      const newlyOpened = [];
      for (const y of neighbors[x]) {
        if (!unvisited.has(y)) continue;

        // Find best open neighbour of y
        let minParent = -1, minCost = Infinity;
        for (const n of neighbors[y]) {
          if (!inOpen[n]) continue;
          if (!this.edgeFree(nodes[n], nodes[y], obstacles)) {
            steps.push({ type: 'edge', subtype: 'collision',
              x1: nodes[n].x, y1: nodes[n].y, x2: nodes[y].x, y2: nodes[y].y });
            continue;
          }
          const c = cost[n] + this.dist(nodes[n], nodes[y]);
          if (c < minCost) { minCost = c; minParent = n; }
        }

        if (minParent !== -1) {
          parent[y] = minParent;
          cost[y]   = minCost;
          newlyOpened.push(y);
          unvisited.delete(y);
          steps.push({ type: 'edge', subtype: 'tree',
            x1: nodes[minParent].x, y1: nodes[minParent].y,
            x2: nodes[y].x, y2: nodes[y].y });
        }
      }

      open.delete(x);
      inOpen[x] = false;
      for (const y of newlyOpened) { open.add(y); inOpen[y] = true; }
    }

    if (parent[1] === -1) return { steps, path: null };
    const path = [];
    let cur = 1;
    while (cur !== -1) { path.unshift({ x: nodes[cur].x, y: nodes[cur].y }); cur = parent[cur]; }
    steps.push({ type: 'path', points: path });
    return { steps, path };
  }
}
Algorithm.register(FMTStarInfo, FMTStarImpl);

// ============================================================
// BIT* – Batch Informed Trees
// ============================================================
class BITStarInfo extends SamplingAlgorithmInfo {
  static id          = 'bitstar';
  static displayName = 'BIT*';
  static description = 'Batch Informed Trees. Iteratively adds batches of samples to a growing random geometric graph, runs heuristic A* search on it, and uses each new solution to shrink the informed sampling region.';
  static paperUrl    = 'https://arxiv.org/abs/1707.01888';
  static paperTitle  = 'Batch Informed Trees (BIT*): Informed asymptotically optimal anytime search (Gammell et al., 2020)';
  static pseudocode  =
`Repeat over MAX_BATCHES:
  Add BATCH_SIZE new samples (informed ellipse once solution exists)
  Build local random geometric graph (CONNECT_RADIUS)
  Run A* on the implicit graph from start to goal
  If better path found: update c_best, shrink ellipse

Return best path found (or failure)`;

  static year               = 2020;
  static timeComplexity     = 'O(n log n) per batch';
  static spaceComplexity    = 'O(n)';
  static optimal            = 'Asymptotically optimal';
  static complete           = 'Probabilistically complete';
  static predecessor        = 'InformedRRT*, FMT*';
  static notableImprovement = 'Combines informed ellipsoidal sampling with batch graph search for rapid anytime path improvement';
  static successor          = null;
  static goodFor            = ['Anytime planning', 'Tightest asymptotic optimality', 'Time-bounded planning'];
  static badFor             = ['Single-shot planning (RRTConnect is faster for first path)', 'Very dynamic environments'];
  static samplingStrategy   = 'batch';
  static queryType          = 'single';
}

class BITStarImpl extends SamplingAlgorithmImpl {
  static id = 'bitstar';

  static MAX_BATCHES    = 6;
  static BATCH_SIZE     = 90;
  static CONNECT_RADIUS = 72;

  /** A* over the implicit geometric graph; emits tree/collision edges to `steps`. */
  static _runAStar(nodes, startIdx, goalIdx, adj, obstacles, steps) {
    const n      = nodes.length;
    const g      = new Array(n).fill(Infinity);
    const f      = new Array(n).fill(Infinity);
    const parent = new Array(n).fill(-1);
    const open   = new Set([startIdx]);
    const closed = new Set();

    g[startIdx] = 0;
    f[startIdx] = this.dist(nodes[startIdx], nodes[goalIdx]);

    while (open.size > 0) {
      let cur = -1, bestF = Infinity;
      for (const idx of open) { if (f[idx] < bestF) { bestF = f[idx]; cur = idx; } }
      if (cur === goalIdx) return { found: true, g, parent };

      open.delete(cur);
      closed.add(cur);

      for (const nb of adj[cur]) {
        if (closed.has(nb)) continue;
        if (!this.edgeFree(nodes[cur], nodes[nb], obstacles)) {
          steps.push({ type: 'edge', subtype: 'collision',
            x1: nodes[cur].x, y1: nodes[cur].y,
            x2: nodes[nb].x, y2: nodes[nb].y });
          continue;
        }
        const tentative = g[cur] + this.dist(nodes[cur], nodes[nb]);
        if (tentative + 1e-6 < g[nb]) {
          parent[nb] = cur;
          g[nb]      = tentative;
          f[nb]      = tentative + this.dist(nodes[nb], nodes[goalIdx]);
          open.add(nb);
          steps.push({ type: 'edge', subtype: 'roadmap',
            x1: nodes[cur].x, y1: nodes[cur].y,
            x2: nodes[nb].x, y2: nodes[nb].y });
        }
      }
    }
    return { found: false, g, parent };
  }

  static _reconstructPath(parent, nodes, goalIdx) {
    if (parent[goalIdx] === -1) return null;
    const path = [];
    let cur = goalIdx;
    while (cur !== -1) { path.unshift({ x: nodes[cur].x, y: nodes[cur].y }); cur = parent[cur]; }
    return path;
  }

  static run({ width, height, obstacles, start, end }) {
    const steps  = [];
    const nodes  = [{ x: start.x, y: start.y }, { x: end.x, y: end.y }];
    let bestPath = null;
    let bestCost = Infinity;

    for (let batch = 0; batch < this.MAX_BATCHES; batch++) {
      for (let i = 0; i < this.BATCH_SIZE; i++) {
        nodes.push(bestPath
          ? this.informedSample(start, end, bestCost, width, height, obstacles)
          : this.randomFreePoint(width, height, obstacles));
      }

      // Build full geometric graph for this batch size
      const adj = Array.from({ length: nodes.length }, () => []);
      for (let i = 0; i < nodes.length; i++) {
        for (let j = i + 1; j < nodes.length; j++) {
          if (this.dist(nodes[i], nodes[j]) <= this.CONNECT_RADIUS) {
            adj[i].push(j);
            adj[j].push(i);
          }
        }
      }

      const res = this._runAStar(nodes, 0, 1, adj, obstacles, steps);
      if (!res.found) continue;
      const path = this._reconstructPath(res.parent, nodes, 1);
      if (!path) continue;
      const cost = res.g[1];
      if (cost < bestCost) { bestCost = cost; bestPath = path; }
    }

    if (!bestPath) return { steps, path: null };
    steps.push({ type: 'path', points: bestPath });
    return { steps, path: bestPath };
  }
}
Algorithm.register(BITStarInfo, BITStarImpl);
