/* ============================================================
   js/main.js  –  Path Finder boilerplate
   ============================================================
   Boilerplate for visualizing path-finding algorithms on a grid.
   Grid constants are read from CSS custom properties so that
   a single source of truth controls the layout.
   ============================================================ */

'use strict';

// ---------------------------------------------------------------------------
// Configuration (mirrors --grid-cols / --grid-rows in styles.css)
// ---------------------------------------------------------------------------
const ROWS = 20;
const COLS = 30;

const SPEED_MAP = {
  slow:   80,   // ms per step
  medium: 30,
  fast:   5,
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
const state = {
  grid: [],          // 2-D array of cell objects
  startNode: null,   // { row, col }
  endNode: null,     // { row, col }
  isMouseDown: false,
  isDragging: null,  // 'start' | 'end' | null
  isRunning: false,
};

// ---------------------------------------------------------------------------
// DOM references
// ---------------------------------------------------------------------------
const gridEl        = document.getElementById('grid');
const algorithmSel  = document.getElementById('algorithm-select');
const speedSel      = document.getElementById('speed-select');
const btnVisualize  = document.getElementById('btn-visualize');
const btnClearPath  = document.getElementById('btn-clear-path');
const btnClearBoard = document.getElementById('btn-clear-board');
const statusMsg     = document.getElementById('status-message');

// ---------------------------------------------------------------------------
// Cell factory
// ---------------------------------------------------------------------------
function createCell(row, col) {
  return {
    row,
    col,
    isWall: false,
    isVisited: false,
    isPath: false,
    isStart: false,
    isEnd: false,
    distance: Infinity,
    heuristic: 0,
    previousNode: null,
  };
}

// ---------------------------------------------------------------------------
// Grid initialisation
// ---------------------------------------------------------------------------
function initGrid() {
  state.grid = [];
  gridEl.innerHTML = '';

  for (let row = 0; row < ROWS; row++) {
    const rowArr = [];
    for (let col = 0; col < COLS; col++) {
      const cell = createCell(row, col);
      rowArr.push(cell);
      gridEl.appendChild(createCellElement(cell));
    }
    state.grid.push(rowArr);
  }

  // Default start / end positions
  setStart(Math.floor(ROWS / 2), Math.floor(COLS * 0.2));
  setEnd(Math.floor(ROWS / 2), Math.floor(COLS * 0.8));

  setStatus('Click cells to place walls, then press Visualize.');
}

// ---------------------------------------------------------------------------
// DOM cell element
// ---------------------------------------------------------------------------
function createCellElement(cell) {
  const el = document.createElement('div');
  el.classList.add('cell');
  el.dataset.row = cell.row;
  el.dataset.col = cell.col;

  el.addEventListener('mousedown', onCellMouseDown);
  el.addEventListener('mouseenter', onCellMouseEnter);
  el.addEventListener('mouseup', onCellMouseUp);

  return el;
}

function getCellElement(row, col) {
  return gridEl.querySelector(`.cell[data-row="${row}"][data-col="${col}"]`);
}

// ---------------------------------------------------------------------------
// State helpers
// ---------------------------------------------------------------------------
function setStart(row, col) {
  if (state.startNode) {
    const prev = state.grid[state.startNode.row][state.startNode.col];
    prev.isStart = false;
    updateCellClass(prev);
  }
  state.startNode = { row, col };
  const cell = state.grid[row][col];
  cell.isStart = true;
  cell.isWall = false;
  updateCellClass(cell);
}

function setEnd(row, col) {
  if (state.endNode) {
    const prev = state.grid[state.endNode.row][state.endNode.col];
    prev.isEnd = false;
    updateCellClass(prev);
  }
  state.endNode = { row, col };
  const cell = state.grid[row][col];
  cell.isEnd = true;
  cell.isWall = false;
  updateCellClass(cell);
}

function updateCellClass(cell) {
  const el = getCellElement(cell.row, cell.col);
  if (!el) return;
  el.className = 'cell';
  if (cell.isStart)   el.classList.add('start');
  else if (cell.isEnd) el.classList.add('end');
  else if (cell.isWall) el.classList.add('wall');
  else if (cell.isPath) el.classList.add('path');
  else if (cell.isVisited) el.classList.add('visited');
}

// ---------------------------------------------------------------------------
// Mouse interaction
// ---------------------------------------------------------------------------
function onCellMouseDown(e) {
  if (state.isRunning) return;
  e.preventDefault();
  state.isMouseDown = true;

  const row = parseInt(e.target.dataset.row);
  const col = parseInt(e.target.dataset.col);
  const cell = state.grid[row][col];

  if (cell.isStart) {
    state.isDragging = 'start';
  } else if (cell.isEnd) {
    state.isDragging = 'end';
  } else {
    toggleWall(row, col);
  }
}

function onCellMouseEnter(e) {
  if (!state.isMouseDown || state.isRunning) return;

  const row = parseInt(e.target.dataset.row);
  const col = parseInt(e.target.dataset.col);

  if (state.isDragging === 'start') {
    setStart(row, col);
  } else if (state.isDragging === 'end') {
    setEnd(row, col);
  } else {
    toggleWall(row, col, true);
  }
}

function onCellMouseUp() {
  state.isMouseDown = false;
  state.isDragging = null;
}

function toggleWall(row, col, forceWall = false) {
  const cell = state.grid[row][col];
  if (cell.isStart || cell.isEnd) return;
  cell.isWall = forceWall ? true : !cell.isWall;
  updateCellClass(cell);
}

// Prevent stray mouse-up outside the grid from locking drag state
document.addEventListener('mouseup', () => {
  state.isMouseDown = false;
  state.isDragging = null;
});

// ---------------------------------------------------------------------------
// Clear helpers
// ---------------------------------------------------------------------------
function clearPath() {
  for (let row = 0; row < ROWS; row++) {
    for (let col = 0; col < COLS; col++) {
      const cell = state.grid[row][col];
      cell.isVisited = false;
      cell.isPath = false;
      cell.distance = Infinity;
      cell.heuristic = 0;
      cell.previousNode = null;
      updateCellClass(cell);
    }
  }
}

function clearBoard() {
  for (let row = 0; row < ROWS; row++) {
    for (let col = 0; col < COLS; col++) {
      const cell = state.grid[row][col];
      cell.isWall = false;
      cell.isVisited = false;
      cell.isPath = false;
      cell.distance = Infinity;
      cell.heuristic = 0;
      cell.previousNode = null;
      updateCellClass(cell);
    }
  }
}

// ---------------------------------------------------------------------------
// Algorithm dispatcher
// ---------------------------------------------------------------------------
async function runAlgorithm() {
  if (state.isRunning) return;
  clearPath();
  state.isRunning = true;
  setStatus('Running…');

  const algorithmName = algorithmSel.value;
  const speed = SPEED_MAP[speedSel.value] ?? SPEED_MAP.medium;
  const start = state.grid[state.startNode.row][state.startNode.col];
  const end   = state.grid[state.endNode.row][state.endNode.col];

  let result;

  switch (algorithmName) {
    case 'bfs':      result = bfs(start, end);      break;
    case 'dfs':      result = dfs(start, end);      break;
    case 'dijkstra': result = dijkstra(start, end); break;
    case 'astar':    result = aStar(start, end);    break;
    default:         result = { visitedInOrder: [], path: [] };
  }

  await animateResult(result.visitedInOrder, result.path, speed);

  state.isRunning = false;
  const found = result.path.length > 0;
  setStatus(found
    ? `Done! Path length: ${result.path.length} cells, visited: ${result.visitedInOrder.length} cells.`
    : 'No path found.');
}

// ---------------------------------------------------------------------------
// Animation helpers
// ---------------------------------------------------------------------------
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function animateResult(visitedInOrder, path, speed) {
  for (const cell of visitedInOrder) {
    if (!cell.isStart && !cell.isEnd) {
      cell.isVisited = true;
      updateCellClass(cell);
      await sleep(speed);
    }
  }

  // Small pause before showing the path
  await sleep(speed * 5);

  for (const cell of path) {
    if (!cell.isStart && !cell.isEnd) {
      cell.isPath = true;
      updateCellClass(cell);
      await sleep(speed * 2);
    }
  }
}

// ---------------------------------------------------------------------------
// Shared utility: get neighbours (4-directional)
// ---------------------------------------------------------------------------
function getNeighbours(cell) {
  const { row, col } = cell;
  const neighbours = [];
  if (row > 0)        neighbours.push(state.grid[row - 1][col]);
  if (row < ROWS - 1) neighbours.push(state.grid[row + 1][col]);
  if (col > 0)        neighbours.push(state.grid[row][col - 1]);
  if (col < COLS - 1) neighbours.push(state.grid[row][col + 1]);
  return neighbours.filter(n => !n.isWall);
}

function retracePath(endCell) {
  const path = [];
  let current = endCell;
  while (current !== null) {
    path.unshift(current);
    current = current.previousNode;
  }
  return path;
}

// ---------------------------------------------------------------------------
// BFS
// ---------------------------------------------------------------------------
function bfs(start, end) {
  const visitedInOrder = [];
  const queue = [start];
  start.isVisited = true;

  while (queue.length > 0) {
    const current = queue.shift();
    visitedInOrder.push(current);
    if (current === end) break;

    for (const neighbour of getNeighbours(current)) {
      if (!neighbour.isVisited) {
        neighbour.isVisited = true;
        neighbour.previousNode = current;
        queue.push(neighbour);
      }
    }
  }

  const path = end.previousNode !== null || end === start ? retracePath(end) : [];
  return { visitedInOrder, path };
}

// ---------------------------------------------------------------------------
// DFS
// ---------------------------------------------------------------------------
function dfs(start, end) {
  const visitedInOrder = [];
  const stack = [start];

  while (stack.length > 0) {
    const current = stack.pop();
    if (current.isVisited) continue;
    current.isVisited = true;
    visitedInOrder.push(current);
    if (current === end) break;

    for (const neighbour of getNeighbours(current)) {
      if (!neighbour.isVisited) {
        neighbour.previousNode = current;
        stack.push(neighbour);
      }
    }
  }

  const path = end.isVisited ? retracePath(end) : [];
  return { visitedInOrder, path };
}

// ---------------------------------------------------------------------------
// Dijkstra
// ---------------------------------------------------------------------------
function dijkstra(start, end) {
  const visitedInOrder = [];
  start.distance = 0;
  const unvisited = getAllNodes();

  while (unvisited.length > 0) {
    sortByDistance(unvisited);
    const closest = unvisited.shift();
    if (closest.isWall) continue;
    if (closest.distance === Infinity) break; // unreachable
    closest.isVisited = true;
    visitedInOrder.push(closest);
    if (closest === end) break;

    for (const neighbour of getNeighbours(closest)) {
      const newDist = closest.distance + 1;
      if (newDist < neighbour.distance) {
        neighbour.distance = newDist;
        neighbour.previousNode = closest;
      }
    }
  }

  const path = end.isVisited ? retracePath(end) : [];
  return { visitedInOrder, path };
}

function getAllNodes() {
  return state.grid.flat();
}

function sortByDistance(nodes) {
  nodes.sort((a, b) => a.distance - b.distance);
}

// ---------------------------------------------------------------------------
// A* Search
// ---------------------------------------------------------------------------
function heuristic(a, b) {
  return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
}

function aStar(start, end) {
  const visitedInOrder = [];
  start.distance  = 0;
  start.heuristic = heuristic(start, end);
  const openSet = [start];

  while (openSet.length > 0) {
    openSet.sort((a, b) => (a.distance + a.heuristic) - (b.distance + b.heuristic));
    const current = openSet.shift();
    if (current.isVisited) continue;
    current.isVisited = true;
    visitedInOrder.push(current);
    if (current === end) break;

    for (const neighbour of getNeighbours(current)) {
      const tentative = current.distance + 1;
      if (tentative < neighbour.distance) {
        neighbour.distance = tentative;
        neighbour.heuristic = heuristic(neighbour, end);
        neighbour.previousNode = current;
        if (!neighbour.isVisited) openSet.push(neighbour);
      }
    }
  }

  const path = end.isVisited ? retracePath(end) : [];
  return { visitedInOrder, path };
}

// ---------------------------------------------------------------------------
// Status helper
// ---------------------------------------------------------------------------
function setStatus(msg) {
  statusMsg.textContent = msg;
}

// ---------------------------------------------------------------------------
// Button wiring
// ---------------------------------------------------------------------------
btnVisualize.addEventListener('click', runAlgorithm);

btnClearPath.addEventListener('click', () => {
  if (state.isRunning) return;
  clearPath();
  setStatus('Path cleared. Click Visualize to run again.');
});

btnClearBoard.addEventListener('click', () => {
  if (state.isRunning) return;
  clearBoard();
  setStatus('Board cleared.');
});

// ---------------------------------------------------------------------------
// Bootstrap
// ---------------------------------------------------------------------------
initGrid();
