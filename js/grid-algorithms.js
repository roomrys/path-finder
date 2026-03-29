'use strict';

/* ============================================================
   js/grid-algorithms.js
   Grid-based pathfinding algorithms.
   Each class extends GridAlgorithm and self-registers.
   ============================================================ */

// ---------------------------------------------------------------------------
// BFS – Breadth-First Search
// ---------------------------------------------------------------------------
class BFS extends GridAlgorithm {
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

  static run(grid, start, end) {
    const visitedInOrder = [];
    const queue = [start];
    start.isVisited = true;

    while (queue.length > 0) {
      const current = queue.shift();
      visitedInOrder.push(current);
      if (current === end) break;

      for (const nb of GridAlgorithm.getNeighbours(grid, current)) {
        if (!nb.isVisited) {
          nb.isVisited = true;
          nb.previousNode = current;
          queue.push(nb);
        }
      }
    }

    const path = (end.previousNode !== null || end === start)
      ? GridAlgorithm.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(BFS);

// ---------------------------------------------------------------------------
// DFS – Depth-First Search
// ---------------------------------------------------------------------------
class DFS extends GridAlgorithm {
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

  static run(grid, start, end) {
    const visitedInOrder = [];
    const stack = [start];

    while (stack.length > 0) {
      const current = stack.pop();
      if (current.isVisited) continue;
      current.isVisited = true;
      visitedInOrder.push(current);
      if (current === end) break;

      for (const nb of GridAlgorithm.getNeighbours(grid, current)) {
        if (!nb.isVisited) {
          nb.previousNode = current;
          stack.push(nb);
        }
      }
    }

    const path = end.isVisited ? GridAlgorithm.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(DFS);

// ---------------------------------------------------------------------------
// Dijkstra – Dijkstra's Shortest-Path Algorithm
// ---------------------------------------------------------------------------
class Dijkstra extends GridAlgorithm {
  static id          = 'dijkstra';
  static displayName = "Dijkstra's Algorithm";
  static description = 'Expands the lowest-cost unvisited node at every step. Optimal for weighted graphs; equivalent to BFS on uniform-cost grids.';
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

      for (const nb of GridAlgorithm.getNeighbours(grid, closest)) {
        const alt = closest.distance + 1;
        if (alt < nb.distance) {
          nb.distance = alt;
          nb.previousNode = closest;
        }
      }
    }

    const path = end.isVisited ? GridAlgorithm.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(Dijkstra);

// ---------------------------------------------------------------------------
// A* – A-Star Search
// ---------------------------------------------------------------------------
class AStar extends GridAlgorithm {
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

  static heuristic(a, b) {
    return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
  }

  static run(grid, start, end) {
    const visitedInOrder = [];
    start.distance  = 0;
    start.heuristic = AStar.heuristic(start, end);
    const openSet   = [start];

    while (openSet.length > 0) {
      openSet.sort((a, b) => (a.distance + a.heuristic) - (b.distance + b.heuristic));
      const current = openSet.shift();
      if (current.isVisited) continue;
      current.isVisited = true;
      visitedInOrder.push(current);
      if (current === end) break;

      for (const nb of GridAlgorithm.getNeighbours(grid, current)) {
        const tentative = current.distance + 1;
        if (tentative < nb.distance) {
          nb.distance     = tentative;
          nb.heuristic    = AStar.heuristic(nb, end);
          nb.previousNode = current;
          if (!nb.isVisited) openSet.push(nb);
        }
      }
    }

    const path = end.isVisited ? GridAlgorithm.retracePath(end) : [];
    return { visitedInOrder, path };
  }
}
Algorithm.register(AStar);
