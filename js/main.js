'use strict';

/* ============================================================
   js/main.js  –  Path Finder orchestration
   Handles grid environment, canvas (free-space) environment,
   animation, UI updates, and algorithm dispatch.
   ============================================================ */

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
const ROWS = 20;
const COLS = 30;

const CANVAS_WIDTH  = 869; // matches 30 * 28 + 29 px gaps
const CANVAS_HEIGHT = 579; // matches 20 * 28 + 19 px gaps

const GRID_SPEED_MAP = { slow: 80, medium: 30, fast: 5 };
// Sampling steps animate faster – algorithms can produce thousands of edges
const SAMPLING_SPEED_MAP = { slow: 10, medium: 3, fast: 0 };

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
const state = {
  // Grid environment
  grid:        [],
  startNode:   null,
  endNode:     null,
  isMouseDown: false,
  isDragging:  null,  // 'start' | 'end' | null

  // Algorithm type
  algorithmType: 'grid', // 'grid' | 'sampling'

  // Canvas environment
  canvasStart:           null,  // {x, y}
  canvasEnd:             null,  // {x, y}
  canvasObstacles:       [],    // [{cx, cy, r}]
  canvasEdges:           [],    // edge steps from current/last run
  canvasPath:            null,  // [{x,y}] | null

  // Canvas mouse interaction
  isMouseDownCanvas:     false,
  isDraggingPoint:       null,  // 'start' | 'end' | null
  isDrawingObstacle:     false,
  obstacleStart:         null,
  currentObstacleRadius: 0,

  // Shared
  isRunning:  false,
  animCancel: null, // { cancelled: false }
};

// ---------------------------------------------------------------------------
// DOM references
// ---------------------------------------------------------------------------
const gridEl          = document.getElementById('grid');
const canvasEl        = document.getElementById('canvas');
const ctx             = canvasEl.getContext('2d');
const algorithmTypeSel = document.getElementById('algorithm-type');
const algorithmSel    = document.getElementById('algorithm-select');
const speedSel        = document.getElementById('speed-select');
const btnVisualize    = document.getElementById('btn-visualize');
const btnClearPath    = document.getElementById('btn-clear-path');
const btnClearBoard   = document.getElementById('btn-clear-board');
const statusMsg       = document.getElementById('status-message');
const footerHint      = document.getElementById('footer-hint');

// ============================================================
// GRID ENVIRONMENT
// ============================================================

function createCell(row, col) {
  return {
    row, col,
    isWall: false, isVisited: false, isPath: false, isStart: false, isEnd: false,
    distance: Infinity, heuristic: 0, previousNode: null,
  };
}

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

  setStart(Math.floor(ROWS / 2), Math.floor(COLS * 0.2));
  setEnd(Math.floor(ROWS / 2), Math.floor(COLS * 0.8));
}

function createCellElement(cell) {
  const el = document.createElement('div');
  el.classList.add('cell');
  el.dataset.row = cell.row;
  el.dataset.col = cell.col;
  el.addEventListener('mousedown',  onCellMouseDown);
  el.addEventListener('mouseenter', onCellMouseEnter);
  el.addEventListener('mouseup',    onCellMouseUp);
  return el;
}

function getCellElement(row, col) {
  return gridEl.querySelector(`.cell[data-row="${row}"][data-col="${col}"]`);
}

function setStart(row, col) {
  if (state.startNode) {
    const prev = state.grid[state.startNode.row][state.startNode.col];
    prev.isStart = false;
    updateCellClass(prev);
  }
  state.startNode = { row, col };
  const cell = state.grid[row][col];
  cell.isStart = true;
  cell.isWall  = false;
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
  cell.isEnd  = true;
  cell.isWall = false;
  updateCellClass(cell);
}

function updateCellClass(cell) {
  const el = getCellElement(cell.row, cell.col);
  if (!el) return;
  el.className = 'cell';
  if      (cell.isStart)   el.classList.add('start');
  else if (cell.isEnd)     el.classList.add('end');
  else if (cell.isWall)    el.classList.add('wall');
  else if (cell.isPath)    el.classList.add('path');
  else if (cell.isVisited) el.classList.add('visited');
}

function onCellMouseDown(e) {
  if (state.isRunning) return;
  e.preventDefault();
  state.isMouseDown = true;
  const row = parseInt(e.target.dataset.row);
  const col = parseInt(e.target.dataset.col);
  const cell = state.grid[row][col];
  if      (cell.isStart) state.isDragging = 'start';
  else if (cell.isEnd)   state.isDragging = 'end';
  else                   toggleWall(row, col);
}

function onCellMouseEnter(e) {
  if (!state.isMouseDown || state.isRunning) return;
  const row = parseInt(e.target.dataset.row);
  const col = parseInt(e.target.dataset.col);
  if      (state.isDragging === 'start') setStart(row, col);
  else if (state.isDragging === 'end')   setEnd(row, col);
  else                                   toggleWall(row, col, true);
}

function onCellMouseUp() {
  state.isMouseDown = false;
  state.isDragging  = null;
}

function toggleWall(row, col, forceWall = false) {
  const cell = state.grid[row][col];
  if (cell.isStart || cell.isEnd) return;
  cell.isWall = forceWall ? true : !cell.isWall;
  updateCellClass(cell);
}

function clearGridPath() {
  for (let r = 0; r < ROWS; r++) {
    for (let c = 0; c < COLS; c++) {
      const cell = state.grid[r][c];
      cell.isVisited = false;
      cell.isPath    = false;
      cell.distance  = Infinity;
      cell.heuristic = 0;
      cell.previousNode = null;
      updateCellClass(cell);
    }
  }
}

function clearGridBoard() {
  for (let r = 0; r < ROWS; r++) {
    for (let c = 0; c < COLS; c++) {
      const cell = state.grid[r][c];
      cell.isWall    = false;
      cell.isVisited = false;
      cell.isPath    = false;
      cell.distance  = Infinity;
      cell.heuristic = 0;
      cell.previousNode = null;
      updateCellClass(cell);
    }
  }
}

// ============================================================
// CANVAS (FREE-SPACE) ENVIRONMENT
// ============================================================

function initCanvas() {
  // Scale for crisp rendering on high-DPI screens
  const dpr = window.devicePixelRatio || 1;
  canvasEl.width  = CANVAS_WIDTH  * dpr;
  canvasEl.height = CANVAS_HEIGHT * dpr;
  canvasEl.style.width  = CANVAS_WIDTH  + 'px';
  canvasEl.style.height = CANVAS_HEIGHT + 'px';
  ctx.scale(dpr, dpr);

  state.canvasStart = { x: Math.round(CANVAS_WIDTH * 0.17), y: Math.round(CANVAS_HEIGHT / 2) };
  state.canvasEnd   = { x: Math.round(CANVAS_WIDTH * 0.83), y: Math.round(CANVAS_HEIGHT / 2) };

  canvasEl.addEventListener('mousedown',   onCanvasMouseDown);
  canvasEl.addEventListener('contextmenu', onCanvasContextMenu);
  renderCanvas();
}

function renderCanvas() {
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // Background
  ctx.fillStyle = '#1e293b';
  ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // Subtle dot grid for spatial reference
  ctx.fillStyle = '#2d3f55';
  for (let x = 40; x < CANVAS_WIDTH; x += 40) {
    for (let y = 40; y < CANVAS_HEIGHT; y += 40) {
      ctx.beginPath();
      ctx.arc(x, y, 1, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Obstacles
  for (const o of state.canvasObstacles) {
    ctx.beginPath();
    ctx.arc(o.cx, o.cy, o.r, 0, Math.PI * 2);
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = '#475569';
    ctx.lineWidth   = 1.5;
    ctx.stroke();
  }

  // In-progress obstacle preview
  if (state.isDrawingObstacle && state.obstacleStart && state.currentObstacleRadius > 2) {
    ctx.beginPath();
    ctx.arc(state.obstacleStart.x, state.obstacleStart.y, state.currentObstacleRadius, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(15,23,42,0.5)';
    ctx.fill();
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth   = 1.5;
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Tree / roadmap edges from last run
  if (state.canvasEdges.length > 0) {
    const isRoadmap = state.canvasEdges[0].subtype === 'roadmap';
    ctx.beginPath();
    ctx.strokeStyle = isRoadmap ? 'rgba(59,130,246,0.3)' : 'rgba(59,130,246,0.65)';
    ctx.lineWidth   = isRoadmap ? 0.8 : 1;
    for (const e of state.canvasEdges) {
      ctx.moveTo(e.x1, e.y1);
      ctx.lineTo(e.x2, e.y2);
    }
    ctx.stroke();
  }

  // Path from last run
  if (state.canvasPath && state.canvasPath.length > 1) {
    ctx.beginPath();
    ctx.moveTo(state.canvasPath[0].x, state.canvasPath[0].y);
    for (let i = 1; i < state.canvasPath.length; i++) {
      ctx.lineTo(state.canvasPath[i].x, state.canvasPath[i].y);
    }
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth   = 3;
    ctx.lineJoin    = 'round';
    ctx.stroke();
    ctx.lineJoin    = 'miter';
  }

  // Start and end markers on top of everything
  drawCirclePoint(state.canvasStart, '#22c55e', 'S');
  drawCirclePoint(state.canvasEnd,   '#ef4444', 'E');
}

function drawCirclePoint(pt, color, label) {
  ctx.beginPath();
  ctx.arc(pt.x, pt.y, 11, 0, Math.PI * 2);
  ctx.fillStyle   = color;
  ctx.fill();
  ctx.strokeStyle = 'rgba(255,255,255,0.8)';
  ctx.lineWidth   = 1.5;
  ctx.stroke();
  ctx.fillStyle     = '#fff';
  ctx.font          = 'bold 10px sans-serif';
  ctx.textAlign     = 'center';
  ctx.textBaseline  = 'middle';
  ctx.fillText(label, pt.x, pt.y);
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function getCanvasPos(e) {
  const rect = canvasEl.getBoundingClientRect();
  return { x: e.clientX - rect.left, y: e.clientY - rect.top };
}

function pointNear(a, b, r) {
  return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 <= r * r;
}

function onCanvasMouseDown(e) {
  if (state.isRunning) return;
  e.preventDefault();
  const pos = getCanvasPos(e);

  if      (pointNear(pos, state.canvasStart, 16)) state.isDraggingPoint = 'start';
  else if (pointNear(pos, state.canvasEnd,   16)) state.isDraggingPoint = 'end';
  else {
    state.isDrawingObstacle     = true;
    state.obstacleStart         = pos;
    state.currentObstacleRadius = 0;
  }
  state.isMouseDownCanvas = true;
}

function onCanvasContextMenu(e) {
  e.preventDefault();
  if (state.isRunning) return;
  const pos = getCanvasPos(e);
  // Remove the topmost obstacle that contains the click point
  for (let i = state.canvasObstacles.length - 1; i >= 0; i--) {
    const o = state.canvasObstacles[i];
    if ((pos.x - o.cx) ** 2 + (pos.y - o.cy) ** 2 <= o.r ** 2) {
      state.canvasObstacles.splice(i, 1);
      renderCanvas();
      return;
    }
  }
}

function clearCanvasPath() {
  state.canvasEdges = [];
  state.canvasPath  = null;
  renderCanvas();
}

function clearCanvasBoard() {
  state.canvasObstacles = [];
  state.canvasEdges     = [];
  state.canvasPath      = null;
  renderCanvas();
}

// Mouse move and up are handled at document level (see below) so drags
// work even when the pointer leaves the canvas element.

// ============================================================
// ANIMATION HELPERS
// ============================================================

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function animateGridResult(visitedInOrder, path, speed, token) {
  for (const cell of visitedInOrder) {
    if (token.cancelled) return;
    if (!cell.isStart && !cell.isEnd) {
      cell.isVisited = true;
      updateCellClass(cell);
      if (speed > 0) await sleep(speed);
    }
  }
  if (token.cancelled) return;
  if (speed > 0) await sleep(speed * 5);

  for (const cell of path) {
    if (token.cancelled) return;
    if (!cell.isStart && !cell.isEnd) {
      cell.isPath = true;
      updateCellClass(cell);
      if (speed > 0) await sleep(speed * 2);
    }
  }
}

async function animateSamplingResult({ steps }, speed, token) {
  state.canvasEdges = [];
  state.canvasPath  = null;

  // Batch size: more edges per frame at higher speed to avoid slow render loops
  const batchSize = speed === 0 ? 50 : speed <= 3 ? 10 : 1;

  let edgeCount = 0;
  for (const step of steps) {
    if (token.cancelled) break;

    if (step.type === 'edge') {
      state.canvasEdges.push(step);
      edgeCount++;
      if (edgeCount % batchSize === 0) {
        renderCanvas();
        if (speed > 0) await sleep(speed);
        else           await sleep(0); // yield to browser
      }
    } else if (step.type === 'path') {
      state.canvasPath = step.points;
      renderCanvas();
      await sleep(300); // brief pause to highlight the path
    }
  }
  renderCanvas(); // ensure final state is drawn
}

// ============================================================
// ALGORITHM RUNNER
// ============================================================

async function runAlgorithm() {
  if (state.isRunning) return;

  const algo = Algorithm.getById(algorithmSel.value);
  if (!algo) return;

  setRunning(true);
  setStatus('Running…');

  const token = { cancelled: false };
  state.animCancel = token;

  try {
    const speedKey = speedSel.value;
    if (state.algorithmType === 'grid') {
      clearGridPath();
      const speed = GRID_SPEED_MAP[speedKey] ?? GRID_SPEED_MAP.medium;
      const start = state.grid[state.startNode.row][state.startNode.col];
      const end   = state.grid[state.endNode.row][state.endNode.col];
      const result = algo.run(state.grid, start, end);
      await animateGridResult(result.visitedInOrder, result.path, speed, token);
      if (!token.cancelled) {
        setStatus(result.path.length > 0
          ? `Done! Path: ${result.path.length} cells  •  Visited: ${result.visitedInOrder.length} cells.`
          : 'No path found.');
      }
    } else {
      clearCanvasPath();
      const speed = SAMPLING_SPEED_MAP[speedKey] ?? SAMPLING_SPEED_MAP.medium;
      const result = algo.run({
        width:     CANVAS_WIDTH,
        height:    CANVAS_HEIGHT,
        obstacles: state.canvasObstacles,
        start:     state.canvasStart,
        end:       state.canvasEnd,
      });
      await animateSamplingResult(result, speed, token);
      if (!token.cancelled) {
        setStatus(result.path
          ? `Done! Path found  (${result.path.length} waypoints,  ${result.steps.filter(s => s.type === 'edge').length} tree edges).`
          : 'No path found — try removing some obstacles or increasing the canvas area.');
      }
    }
  } finally {
    setRunning(false);
    state.animCancel = null;
  }
}

// ============================================================
// UI MANAGEMENT
// ============================================================

function setRunning(running) {
  state.isRunning         = running;
  btnVisualize.disabled   = running;
  algorithmSel.disabled   = running;
  algorithmTypeSel.disabled = running;
  speedSel.disabled       = running;
}

function setStatus(msg) {
  statusMsg.textContent = msg;
}

function populateAlgorithmDropdown(type) {
  algorithmSel.innerHTML = '';
  for (const algo of Algorithm.getByType(type)) {
    const opt     = document.createElement('option');
    opt.value     = algo.id;
    opt.textContent = algo.displayName;
    algorithmSel.appendChild(opt);
  }
  const first = Algorithm.getByType(type)[0];
  if (first) updateInfoPanel(first);
}

function switchAlgorithmType(type) {
  state.algorithmType = type;

  if (type === 'grid') {
    gridEl.classList.remove('hidden');
    canvasEl.classList.add('hidden');
    document.getElementById('grid-legend').classList.remove('hidden');
    document.getElementById('sampling-legend').classList.add('hidden');
    setStatus('Click cells to place walls, then press Visualize.');
  } else {
    gridEl.classList.add('hidden');
    canvasEl.classList.remove('hidden');
    document.getElementById('grid-legend').classList.add('hidden');
    document.getElementById('sampling-legend').classList.remove('hidden');
    renderCanvas();
    setStatus('Click & drag to place obstacles, then press Visualize.');
  }

  populateAlgorithmDropdown(type);
  updateFooterHint(type);
}

function updateFooterHint(type) {
  if (!footerHint) return;
  footerHint.textContent = type === 'grid'
    ? 'Click cells to toggle walls  \u2022  Drag start/end nodes to reposition them'
    : 'Click & drag to place obstacles  \u2022  Right-click obstacle to remove  \u2022  Drag start/end to reposition';
}

function updateInfoPanel(algo) {
  if (!algo) return;

  document.getElementById('info-name').textContent = algo.displayName;
  document.getElementById('info-description').textContent = algo.description;

  const paperLink = document.getElementById('info-paper');
  if (algo.paperUrl) {
    paperLink.href        = algo.paperUrl;
    paperLink.textContent = (algo.paperTitle || 'Read Paper') + ' \u2192';
    paperLink.classList.remove('hidden');
  } else {
    paperLink.classList.add('hidden');
  }

  document.getElementById('pseudocode-content').textContent = algo.pseudocode;

  const implHeader = `// ${algo.displayName} — run() implementation\n\n`;
  document.getElementById('implementation-content').textContent = implHeader + algo.run.toString();
}

// ============================================================
// DOCUMENT-LEVEL MOUSE EVENTS  (handles drags that leave the canvas)
// ============================================================

document.addEventListener('mousemove', e => {
  // Grid drag
  if (state.isMouseDown && state.isDragging && state.algorithmType === 'grid') {
    // handled by onCellMouseEnter
    return;
  }

  // Canvas drag / draw
  if (!state.isMouseDownCanvas) return;
  const rect = canvasEl.getBoundingClientRect();
  const x = e.clientX - rect.left;
  const y = e.clientY - rect.top;

  if (state.isDraggingPoint === 'start') {
    state.canvasStart = { x: clamp(x, 0, CANVAS_WIDTH), y: clamp(y, 0, CANVAS_HEIGHT) };
    renderCanvas();
  } else if (state.isDraggingPoint === 'end') {
    state.canvasEnd = { x: clamp(x, 0, CANVAS_WIDTH), y: clamp(y, 0, CANVAS_HEIGHT) };
    renderCanvas();
  } else if (state.isDrawingObstacle && state.obstacleStart) {
    const dx = x - state.obstacleStart.x;
    const dy = y - state.obstacleStart.y;
    state.currentObstacleRadius = Math.sqrt(dx * dx + dy * dy);
    renderCanvas();
  }
});

document.addEventListener('mouseup', () => {
  // Grid
  state.isMouseDown = false;
  state.isDragging  = null;

  // Canvas – finalise obstacle if still drawing
  if (state.isDrawingObstacle) {
    if (state.currentObstacleRadius > 8) {
      state.canvasObstacles.push({
        cx: state.obstacleStart.x,
        cy: state.obstacleStart.y,
        r:  state.currentObstacleRadius,
      });
    }
    state.isDrawingObstacle     = false;
    state.obstacleStart         = null;
    state.currentObstacleRadius = 0;
    if (state.algorithmType === 'sampling') renderCanvas();
  }
  state.isMouseDownCanvas = false;
  state.isDraggingPoint   = null;
});

// ============================================================
// BUTTON WIRING
// ============================================================

btnVisualize.addEventListener('click', runAlgorithm);

btnClearPath.addEventListener('click', () => {
  if (state.isRunning) return;
  if (state.algorithmType === 'grid') clearGridPath();
  else                                clearCanvasPath();
  setStatus('Path cleared.');
});

btnClearBoard.addEventListener('click', () => {
  // Cancel any in-progress animation
  if (state.animCancel) state.animCancel.cancelled = true;
  setRunning(false);

  if (state.algorithmType === 'grid') clearGridBoard();
  else                                clearCanvasBoard();
  setStatus('Board cleared.');
});

algorithmTypeSel.addEventListener('change', () => {
  if (state.isRunning) return;
  switchAlgorithmType(algorithmTypeSel.value);
});

algorithmSel.addEventListener('change', () => {
  const algo = Algorithm.getById(algorithmSel.value);
  if (algo) updateInfoPanel(algo);
});

// Reset tab to pseudocode when algorithm changes
algorithmSel.addEventListener('change', () => {
  document.querySelectorAll('.tab-btn').forEach((b, i) => b.classList.toggle('active', i === 0));
  document.getElementById('tab-pseudocode').classList.remove('hidden');
  document.getElementById('tab-implementation').classList.add('hidden');
});

document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(c => c.classList.add('hidden'));
    btn.classList.add('active');
    document.getElementById(`tab-${btn.dataset.tab}`).classList.remove('hidden');
  });
});

// ============================================================
// BOOTSTRAP
// ============================================================
initGrid();
initCanvas();
switchAlgorithmType('grid');
