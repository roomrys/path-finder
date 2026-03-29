'use strict';

/* ============================================================
   js/grid-algorithms.js
   Each algorithm is defined as two classes:
     [Name]Info  extends GridAlgorithmInfo  — metadata only
     [Name]Impl  extends GridAlgorithmImpl  — run() only
   Paired via Algorithm.register(Info, Impl).
   ============================================================ */

// ============================================================
// BFS – Breadth-First Search
// ============================================================
class BFSInfo extends GridAlgorithmInfo {
  static id          = 'bfs';
  static displayName = 'Breadth-First Search (BFS)';
  static description = 'Explores all neighbours level-by-level using a queue. Guarantees the shortest path on unweighted grids.';
  static pseudocode  =
`Initialize queue Q <- [start]
Mark start as visited

While Q is not empty:
  node <- dequeue(Q)
  If node is goal: return path

  For each unvisited non-wall neighbour n of node:
    n.parent <- node
    Mark n as visited
    Enqueue n into Q

Return  no path found`;

  static year               = 1959;
  static timeComplexity     = 'O(V + E)';
  static spaceComplexity    = 'O(V)';
  static optimal            = 'Yes — shortest path on unweighted graphs';
  static complete           = 'Yes';
  static predecessor        = null;
  static notableImprovement = null;
  static successor          = "Dijkstra's Algorithm (adds edge weights)";
  static goodFor            = ['Unweighted grids', 'Guaranteed shortest path', 'Small to medium maps'];
  static badFor             = ['Weighted graphs (use Dijkstra)', 'Memory-constrained environments'];
  static graphType          = 'Unweighted';
  static heuristic          = null;
}

class BFSImpl extends GridAlgorithmImpl {
  static id = 'bfs';

  static run(grid, start, end) {
    const visitedInOrder = [];
    const queue = [start];
    start.isVisited = true;

    while (queue.length > 0) {
      const current = queue.shift();
      visitedInOrder.push(current);
      if (current === end) break;

      for (const nb of GridAlgorithmImpl.getNeighbours(grid, current)) {
        if (!nb.isVisited) {
          nb.isVisited = true;
          nb.previousNode = current;
          queue.push(nb);
        }
      }
    }

    const path = (end.previousNode !== null || end === start)
      ? GridAlgorithmImpl.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(BFSInfo, BFSImpl);

// ============================================================
// DFS – Depth-First Search
// ============================================================
class DFSInfo extends GridAlgorithmInfo {
  static id          = 'dfs';
  static displayName = 'Depth-First Search (DFS)';
  static description = 'Explores as far as possible along each branch before backtracking. Does not guarantee the shortest path.';
  static pseudocode  =
`Initialize stack S <- [start]

While S is not empty:
  node <- pop(S)
  If node already visited: continue
  Mark node as visited
  If node is goal: return path

  For each unvisited non-wall neighbour n of node:
    n.parent <- node
    Push n onto S

Return  no path found`;

  static year               = 1959;
  static timeComplexity     = 'O(V + E)';
  static spaceComplexity    = 'O(V)';
  static optimal            = 'No — does not guarantee shortest path';
  static complete           = 'Yes — on finite graphs';
  static predecessor        = null;
  static notableImprovement = null;
  static successor          = 'Iterative Deepening DFS (IDDFS)';
  static goodFor            = ['Exploring all reachable cells', 'Detecting whether a path exists', 'Low overhead'];
  static badFor             = ['Finding shortest paths', 'Large or infinite graphs'];
  static graphType          = 'Unweighted';
  static heuristic          = null;
}

class DFSImpl extends GridAlgorithmImpl {
  static id = 'dfs';

  static run(grid, start, end) {
    const visitedInOrder = [];
    const stack = [start];

    while (stack.length > 0) {
      const current = stack.pop();
      if (current.isVisited) continue;
      current.isVisited = true;
      visitedInOrder.push(current);
      if (current === end) break;

      for (const nb of GridAlgorithmImpl.getNeighbours(grid, current)) {
        if (!nb.isVisited) {
          nb.previousNode = current;
          stack.push(nb);
        }
      }
    }

    const path = end.isVisited ? GridAlgorithmImpl.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(DFSInfo, DFSImpl);

// ============================================================
// Dijkstra – Dijkstra's Shortest-Path Algorithm
// ============================================================
class DijkstraInfo extends GridAlgorithmInfo {
  static id          = 'dijkstra';
  static displayName = "Dijkstra's Algorithm";
  static description = "Expands the lowest-cost unvisited node at every step. Optimal for weighted graphs; equivalent to BFS on uniform-cost grids.";
  static paperUrl    = 'https://doi.org/10.1007/BF01386390';
  static paperTitle  = 'A Note on Two Problems in Connexion with Graphs (Dijkstra, 1959)';
  static pseudocode  =
`Set dist[start] <- 0,  dist[all others] <- infinity

While unvisited nodes remain:
  u <- unvisited node with minimum dist[u]
  If u is goal: return path
  If dist[u] = infinity: break  (goal unreachable)
  Mark u as visited

  For each unvisited neighbour v of u:
    alt <- dist[u] + weight(u, v)
    If alt < dist[v]:
      dist[v] <- alt
      v.parent <- u

Return  no path found`;

  static year               = 1959;
  static timeComplexity     = 'O((V + E) log V)';
  static spaceComplexity    = 'O(V)';
  static optimal            = 'Yes — shortest path on non-negative weighted graphs';
  static complete           = 'Yes';
  static predecessor        = 'BFS';
  static notableImprovement = 'Handles weighted edges; extends BFS to general non-negative cost graphs';
  static successor          = 'A* Search (adds heuristic guidance)';
  static goodFor            = ['Weighted graphs', 'Guaranteed shortest path', 'No admissible heuristic available'];
  static badFor             = ['Unweighted graphs (BFS is faster)', 'Very large graphs without a heuristic (A* is faster)', 'Negative edge weights'];
  static graphType          = 'Weighted';
  static heuristic          = null;
}

class DijkstraImpl extends GridAlgorithmImpl {
  static id = 'dijkstra';

  static run(grid, start, end) {
    const visitedInOrder = [];
    start.distance = 0;
    const unvisited = grid.flat();

    while (unvisited.length > 0) {
      unvisited.sort((a, b) => a.distance - b.distance);
      const closest = unvisited.shift();
      if (closest.isWall) continue;
      if (closest.distance === Infinity) break;
      closest.isVisited = true;
      visitedInOrder.push(closest);
      if (closest === end) break;

      for (const nb of GridAlgorithmImpl.getNeighbours(grid, closest)) {
        const alt = closest.distance + 1;
        if (alt < nb.distance) {
          nb.distance     = alt;
          nb.previousNode = closest;
        }
      }
    }

    const path = end.isVisited ? GridAlgorithmImpl.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(DijkstraInfo, DijkstraImpl);

// ============================================================
// A* – A-Star Search
// ============================================================
class AStarInfo extends GridAlgorithmInfo {
  static id          = 'astar';
  static displayName = 'A* Search';
  static description = 'Combines actual path cost with a Manhattan-distance heuristic to guide the search. Optimal and typically explores far fewer nodes than Dijkstra.';
  static pseudocode  =
`h(n) = Manhattan distance from n to goal

Set g[start] <- 0,  f[start] <- h(start)
Open set O <- {start}

While O is not empty:
  current <- node in O with minimum f[current]
  If current is goal: return path
  Move current to closed set

  For each neighbour n of current:
    If n in closed set: skip
    tentative_g <- g[current] + 1
    If tentative_g < g[n]:
      n.parent  <- current
      g[n]      <- tentative_g
      f[n]      <- g[n] + h(n)
      Add n to O if not already there

Return  no path found`;

  static year               = 1968;
  static timeComplexity     = 'O(E log V)';
  static spaceComplexity    = 'O(V)';
  static optimal            = 'Yes — with admissible heuristic';
  static complete           = 'Yes';
  static predecessor        = "Dijkstra's Algorithm";
  static notableImprovement = 'Heuristic guidance drastically reduces the number of nodes explored';
  static successor          = 'Weighted A*, D*, Theta*';
  static goodFor            = ['Known goal position', 'Grid pathfinding with admissible heuristic', 'Best performance among uninformed/informed grid algorithms'];
  static badFor             = ['Unknown goal location', 'Very large state spaces without a tight heuristic', 'Memory-limited settings (use IDA*)'];
  static graphType          = 'Weighted';
  static heuristic          = 'Manhattan distance';
}

class AStarImpl extends GridAlgorithmImpl {
  static id = 'astar';

  static heuristic(a, b) {
    return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
  }

  static run(grid, start, end) {
    const visitedInOrder = [];
    start.distance  = 0;
    start.heuristic = AStarImpl.heuristic(start, end);
    const openSet   = [start];

    while (openSet.length > 0) {
      openSet.sort((a, b) => (a.distance + a.heuristic) - (b.distance + b.heuristic));
      const current = openSet.shift();
      if (current.isVisited) continue;
      current.isVisited = true;
      visitedInOrder.push(current);
      if (current === end) break;

      for (const nb of GridAlgorithmImpl.getNeighbours(grid, current)) {
        const tentative = current.distance + 1;
        if (tentative < nb.distance) {
          nb.distance     = tentative;
          nb.heuristic    = AStarImpl.heuristic(nb, end);
          nb.previousNode = current;
          if (!nb.isVisited) openSet.push(nb);
        }
      }
    }

    const path = end.isVisited ? GridAlgorithmImpl.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(AStarInfo, AStarImpl);
