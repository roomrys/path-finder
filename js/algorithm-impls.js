'use strict';

/* ============================================================
   js/algorithm-impls.js

   AlgorithmImpl        — pure implementation base class.
   GridAlgorithmImpl    — grid helpers shared by all grid algorithms.
   SamplingAlgorithmImpl — geometry helpers shared by all sampling algorithms.

   No metadata lives here. For display info see AlgorithmInfo in
   algorithm-registry.js.
   ============================================================ */

// ---------------------------------------------------------------------------
// AlgorithmImpl – base implementation class
// ---------------------------------------------------------------------------
class AlgorithmImpl {
  static id = ''; // must match the paired AlgorithmInfo.id
}

// ---------------------------------------------------------------------------
// GridAlgorithmImpl – shared helpers for grid-based algorithms
//
// run(grid, start, end) -> { visitedInOrder: Cell[], path: Cell[] }
// ---------------------------------------------------------------------------
class GridAlgorithmImpl extends AlgorithmImpl {
  /** Returns walkable (non-wall) 4-directional neighbours of `cell`. */
  static getNeighbours(grid, cell) {
    const { row, col } = cell;
    const rows = grid.length, cols = grid[0].length;
    const ns = [];
    if (row > 0)      ns.push(grid[row - 1][col]);
    if (row < rows-1) ns.push(grid[row + 1][col]);
    if (col > 0)      ns.push(grid[row][col - 1]);
    if (col < cols-1) ns.push(grid[row][col + 1]);
    return ns.filter(n => !n.isWall);
  }

  /** Reconstructs a path by following `previousNode` pointers back to root. */
  static retracePath(endCell) {
    const path = [];
    let cur = endCell;
    while (cur !== null) { path.unshift(cur); cur = cur.previousNode; }
    return path;
  }
}

// ---------------------------------------------------------------------------
// SamplingAlgorithmImpl – shared helpers for sampling-based algorithms
//
// run(env) -> { steps: Step[], path: Point[]|null }
//   env  = { width, height, obstacles: {cx,cy,r}[], start: {x,y}, end: {x,y} }
//   Step = { type: 'edge', subtype: 'tree'|'roadmap'|'collision', x1,y1,x2,y2 }
//        | { type: 'path', points: {x,y}[] }
// ---------------------------------------------------------------------------
class SamplingAlgorithmImpl extends AlgorithmImpl {
  /** Euclidean distance between two points. */
  static dist(a, b) {
    return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
  }

  /** Move `stepSize` units from `from` toward `to`. */
  static steer(from, to, stepSize) {
    const d = this.dist(from, to);
    if (d <= stepSize) return { x: to.x, y: to.y };
    const t = stepSize / d;
    return { x: from.x + (to.x - from.x) * t, y: from.y + (to.y - from.y) * t };
  }

  /** Returns true if `point` is not inside any obstacle. */
  static isFree(point, obstacles) {
    for (const o of obstacles) {
      if ((point.x - o.cx) ** 2 + (point.y - o.cy) ** 2 <= o.r ** 2) return false;
    }
    return true;
  }

  /** Returns true if the segment p1->p2 does not intersect any obstacle. */
  static edgeFree(p1, p2, obstacles) {
    for (const o of obstacles) {
      if (this._segIntersectsCircle(p1.x, p1.y, p2.x, p2.y, o.cx, o.cy, o.r)) return false;
    }
    return true;
  }

  /** Returns a uniformly random point in [0,width) x [0,height). */
  static randomPoint(width, height) {
    return { x: Math.random() * width, y: Math.random() * height };
  }

  /** Returns a random point in free space (up to 200 rejection-sampling attempts). */
  static randomFreePoint(width, height, obstacles) {
    for (let i = 0; i < 200; i++) {
      const pt = this.randomPoint(width, height);
      if (this.isFree(pt, obstacles)) return pt;
    }
    return this.randomPoint(width, height); // fallback
  }

  /** Returns the index of the node in `nodes` nearest to `target`. */
  static nearestIdx(nodes, target) {
    let bestIdx = 0, bestD2 = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d2 = (nodes[i].x - target.x) ** 2 + (nodes[i].y - target.y) ** 2;
      if (d2 < bestD2) { bestD2 = d2; bestIdx = i; }
    }
    return bestIdx;
  }

  /** Reconstructs a point path by following `parentIdx` from `idx` to root. */
  static tracePath(nodes, idx) {
    const out = [];
    let cur = idx;
    while (cur !== -1) {
      out.unshift({ x: nodes[cur].x, y: nodes[cur].y });
      cur = nodes[cur].parentIdx;
    }
    return out;
  }

  /**
   * Samples from the informed prolate hyperspheroid (2-D ellipse) once a
   * solution of cost `cBest` is known. Falls back to uniform free-space
   * sampling when no finite bound exists yet.
   */
  static informedSample(start, end, cBest, width, height, obstacles) {
    const cMin = this.dist(start, end);
    if (!Number.isFinite(cBest) || cBest <= cMin + 1e-6) {
      return this.randomFreePoint(width, height, obstacles);
    }
    const a  = cBest / 2;
    const b  = Math.sqrt(Math.max(0, cBest * cBest - cMin * cMin)) / 2;
    const cx = (start.x + end.x) / 2;
    const cy = (start.y + end.y) / 2;
    const theta = Math.atan2(end.y - start.y, end.x - start.x);
    const ct = Math.cos(theta), st = Math.sin(theta);
    for (let i = 0; i < 120; i++) {
      const r   = Math.sqrt(Math.random());
      const phi = Math.random() * 2 * Math.PI;
      const ex  = a * r * Math.cos(phi);
      const ey  = b * r * Math.sin(phi);
      const x   = cx + ex * ct - ey * st;
      const y   = cy + ex * st + ey * ct;
      const p   = { x, y };
      if (x >= 0 && x <= width && y >= 0 && y <= height && this.isFree(p, obstacles)) return p;
    }
    return this.randomFreePoint(width, height, obstacles);
  }

  /** Parametric line-segment / circle intersection test. */
  static _segIntersectsCircle(x1, y1, x2, y2, cx, cy, r) {
    const dx = x2 - x1, dy = y2 - y1;
    const fx = x1 - cx, fy = y1 - cy;
    const a  = dx * dx + dy * dy;
    const b  = 2 * (fx * dx + fy * dy);
    const c  = fx * fx + fy * fy - r * r;
    let disc = b * b - 4 * a * c;
    if (disc < 0) return false;
    disc = Math.sqrt(disc);
    const t1 = (-b - disc) / (2 * a);
    const t2 = (-b + disc) / (2 * a);
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t1 < 0 && t2 > 1);
  }
}
