'use strict';

/* ============================================================
   js/main.js  –  Path Finder orchestration
   ============================================================ */

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
const ROWS = 20;
const COLS = 30;

let CANVAS_WIDTH  = 0; // set dynamically by resizeCanvas()
let CANVAS_HEIGHT = 0;

const GRID_SPEED_MAP     = { slow: 80,  medium: 30, fast: 5 };
const SAMPLING_SPEED_MAP = { slow: 10,  medium: 3,  fast: 0 };

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
const state = {
  // Grid environment
  grid:        [],
  startNode:   null,
  endNode:     null,
  isMouseDown: false,
  isDragging:  null,   // 'start' | 'end' | null

  // Algorithm type
  algorithmType: 'sampling', // 'grid' | 'sampling'

  // Canvas environment
  canvasStart:           null,  // {x, y}
  canvasEnd:             null,  // {x, y}
  canvasObstacles:       [],    // [{cx, cy, r}]
  canvasTreeEdges:       [],    // subtype 'tree'
  canvasRoadmapEdges:    [],    // subtype 'roadmap'
  canvasCollisionEdges:  [],    // subtype 'collision'
  canvasPath:            null,  // [{x,y}] | null

  // Canvas mouse interaction
  isMouseDownCanvas:     false,
  isDraggingPoint:       null,  // 'start' | 'end' | null
  isDrawingObstacle:     false,
  obstacleStart:         null,
  currentObstacleRadius: 0,

  // Shared
  isRunning:  false,
  animCancel: null,   // { cancelled: false }
};

// ---------------------------------------------------------------------------
// DOM references
// ---------------------------------------------------------------------------
const gridEl           = document.getElementById('grid');
const canvasEl         = document.getElementById('canvas');
const ctx              = canvasEl.getContext('2d');
const algorithmTypeSel = document.getElementById('algorithm-type');
const algorithmSel     = document.getElementById('algorithm-select');
const speedSel         = document.getElementById('speed-select');
const btnVisualize     = document.getElementById('btn-visualize');
const btnClearBoard    = document.getElementById('btn-clear-board');
const statusMsg        = document.getElementById('status-message');
const footerHint       = document.getElementById('footer-hint');

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
  const row  = parseInt(e.target.dataset.row);
  const col  = parseInt(e.target.dataset.col);
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

function resizeCanvas() {
  const w = canvasEl.clientWidth;
  const h = canvasEl.clientHeight;
  if (w === 0 || h === 0) return; // not visible yet

  CANVAS_WIDTH  = w;
  CANVAS_HEIGHT = h;
  // Set buffer to CSS pixel size (1:1 mapping — no DPR scaling needed).
  // Setting canvas.width also resets the context transform to identity.
  canvasEl.width  = w;
  canvasEl.height = h;

  if (state.canvasStart) {
    // Clamp existing points to new bounds
    state.canvasStart.x = clamp(state.canvasStart.x, 0, w);
    state.canvasStart.y = clamp(state.canvasStart.y, 0, h);
    state.canvasEnd.x   = clamp(state.canvasEnd.x,   0, w);
    state.canvasEnd.y   = clamp(state.canvasEnd.y,   0, h);
  } else {
    state.canvasStart = { x: Math.round(w * 0.17), y: Math.round(h / 2) };
    state.canvasEnd   = { x: Math.round(w * 0.83), y: Math.round(h / 2) };
  }
  renderCanvas();
}

function initCanvas() {
  canvasEl.addEventListener('mousedown',   onCanvasMouseDown);
  canvasEl.addEventListener('contextmenu', onCanvasContextMenu);
}

function renderCanvas() {
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // Background
  ctx.fillStyle = '#1e293b';
  ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // Dot grid
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
    ctx.fillStyle   = '#0f172a';
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

  // Collision edges (drawn first so tree edges appear on top)
  if (state.canvasCollisionEdges.length > 0) {
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(239,68,68,0.65)';
    ctx.lineWidth   = 1.5;
    for (const e of state.canvasCollisionEdges) {
      ctx.moveTo(e.x1, e.y1);
      ctx.lineTo(e.x2, e.y2);
    }
    ctx.stroke();
  }

  // Roadmap edges
  if (state.canvasRoadmapEdges.length > 0) {
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(59,130,246,0.3)';
    ctx.lineWidth   = 0.8;
    for (const e of state.canvasRoadmapEdges) {
      ctx.moveTo(e.x1, e.y1);
      ctx.lineTo(e.x2, e.y2);
    }
    ctx.stroke();
  }

  // Tree edges
  if (state.canvasTreeEdges.length > 0) {
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(59,130,246,0.65)';
    ctx.lineWidth   = 1;
    for (const e of state.canvasTreeEdges) {
      ctx.moveTo(e.x1, e.y1);
      ctx.lineTo(e.x2, e.y2);
    }
    ctx.stroke();
  }

  // Path
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

  // Start / end markers on top
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
  ctx.fillStyle    = '#fff';
  ctx.font         = 'bold 10px sans-serif';
  ctx.textAlign    = 'center';
  ctx.textBaseline = 'middle';
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
  e.preventDefault();
  const pos = getCanvasPos(e);

  if (state.isRunning) {
    // While running: only allow obstacle drawing (not point dragging)
    if (!pointNear(pos, state.canvasStart, 16) && !pointNear(pos, state.canvasEnd, 16)) {
      state.isDrawingObstacle     = true;
      state.obstacleStart         = pos;
      state.currentObstacleRadius = 0;
      state.isMouseDownCanvas     = true;
    }
    return;
  }

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
  // Right-click removes topmost obstacle — allowed even while running
  const pos = getCanvasPos(e);
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
  state.canvasTreeEdges      = [];
  state.canvasRoadmapEdges   = [];
  state.canvasCollisionEdges = [];
  state.canvasPath           = null;
  renderCanvas();
}

function clearCanvasBoard() {
  state.canvasObstacles      = [];
  state.canvasTreeEdges      = [];
  state.canvasRoadmapEdges   = [];
  state.canvasCollisionEdges = [];
  state.canvasPath           = null;
  renderCanvas();
}

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

// Speed is read live from speedSel each frame so the user can change it while running.
async function animateSamplingResult({ steps }, token) {
  state.canvasTreeEdges      = [];
  state.canvasRoadmapEdges   = [];
  state.canvasCollisionEdges = [];
  state.canvasPath           = null;

  let edgeCount = 0;

  for (const step of steps) {
    if (token.cancelled) break;

    const speed     = SAMPLING_SPEED_MAP[speedSel.value] ?? SAMPLING_SPEED_MAP.medium;
    const batchSize = speed === 0 ? 100 : speed <= 3 ? 15 : 5;

    if (step.type === 'edge') {
      if      (step.subtype === 'tree')      state.canvasTreeEdges.push(step);
      else if (step.subtype === 'roadmap')   state.canvasRoadmapEdges.push(step);
      else if (step.subtype === 'collision') state.canvasCollisionEdges.push(step);
      edgeCount++;
      if (edgeCount % batchSize === 0) {
        renderCanvas();
        if (speed > 0) await sleep(speed);
        else           await sleep(0); // yield to browser even at max speed
      }
    } else if (step.type === 'path') {
      state.canvasPath = step.points;
      renderCanvas();
      await sleep(300);
    }
  }
  renderCanvas(); // final frame
}

// ============================================================
// ALGORITHM RUNNER
// ============================================================

async function runAlgorithm() {
  if (state.isRunning) return;

  const entry = Algorithm.getById(algorithmSel.value);
  if (!entry) return;

  setRunning(true);
  setStatus('Running\u2026');
  switchToTab('visualizer');

  const token    = { cancelled: false };
  state.animCancel = token;

  try {
    const speedKey = speedSel.value;
    if (state.algorithmType === 'grid') {
      clearGridPath();
      const speed  = GRID_SPEED_MAP[speedKey] ?? GRID_SPEED_MAP.medium;
      const start  = state.grid[state.startNode.row][state.startNode.col];
      const end    = state.grid[state.endNode.row][state.endNode.col];
      const result = entry.impl.run(state.grid, start, end);
      await animateGridResult(result.visitedInOrder, result.path, speed, token);
      if (!token.cancelled) {
        setStatus(result.path.length > 0
          ? `Done! Path: ${result.path.length} cells  \u2022  Visited: ${result.visitedInOrder.length} cells.`
          : 'No path found.');
      }
    } else {
      clearCanvasPath();
      const result = entry.impl.run({
        width:     CANVAS_WIDTH,
        height:    CANVAS_HEIGHT,
        obstacles: state.canvasObstacles,
        start:     state.canvasStart,
        end:       state.canvasEnd,
      });
      await animateSamplingResult(result, token);
      if (!token.cancelled) {
        const nEdges = state.canvasTreeEdges.length + state.canvasRoadmapEdges.length;
        setStatus(result.path
          ? `Done! Path found  (${result.path.length} waypoints,  ${nEdges} edges,  ${state.canvasCollisionEdges.length} collisions).`
          : 'No path found \u2014 try removing some obstacles or running again.');
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

/** Cancel any in-progress run without clearing the board. */
function cancelRun() {
  if (state.animCancel) state.animCancel.cancelled = true;
  setRunning(false);
  state.animCancel = null;
}

function setRunning(running) {
  state.isRunning       = running;
  btnVisualize.disabled = running;
  // Speed and obstacle interaction remain live during a run.
  // Algorithm/type selects are not disabled – changes cancel the run instead.
}

function setStatus(msg) {
  statusMsg.textContent = msg;
}

function populateAlgorithmDropdown(type) {
  algorithmSel.innerHTML = '';
  for (const entry of Algorithm.getAllOfType(type)) {
    const opt       = document.createElement('option');
    opt.value       = entry.info.id;
    opt.textContent = entry.info.displayName;
    algorithmSel.appendChild(opt);
  }
  const entries = Algorithm.getAllOfType(type);
  if (entries.length) updateInfoPanel(entries[0].info, entries[0].impl);
}

function switchAlgorithmType(type) {
  state.algorithmType       = type;
  algorithmTypeSel.value    = type;

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
    resizeCanvas();
    setStatus('Click & drag to place obstacles, then press Visualize.');
  }


  populateAlgorithmDropdown(type);
  updateFooterHint(type);
}

function updateFooterHint(type) {
  if (!footerHint) return;
  footerHint.textContent = type === 'grid'
    ? 'Click cells to toggle walls  \u2022  Drag start/end nodes to reposition them'
    : 'Click & drag to draw obstacles  \u2022  Right-click to remove  \u2022  Drag start/end to reposition  \u2022  Speed & obstacles can be changed while running';
}

// ---------------------------------------------------------------------------
// Minimal JS syntax highlighter for the Implementation tab
// ---------------------------------------------------------------------------
function highlightJS(raw) {
  // Escape HTML entities so injected source can't break the DOM
  const safe = raw
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');

  // Single-pass tokeniser — patterns checked in priority order via alternation
  const TOKEN_RE = new RegExp([
    '(\\/\\*[\\s\\S]*?\\*\\/)',                                        // block comment
    '(\\/\\/[^\\n]*)',                                                  // line comment
    '(`(?:\\\\[\\s\\S]|[^`])*`)',                                      // template literal
    '("(?:\\\\[\\s\\S]|[^"\\n])*")',                                   // double-quoted string
    "('(?:\\\\[\\s\\S]|[^'\\n])*')",                                   // single-quoted string
    '\\b(const|let|var|function|class|static|return|if|else|for|' +
      'while|break|continue|new|this|of|in|true|false|null|' +
      'undefined|Infinity|typeof|instanceof|async|await)\\b',          // keywords
    '\\b(0x[0-9a-fA-F]+|\\d+(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)\\b',     // numbers
    '\\b([A-Za-z_$][A-Za-z0-9_$]*)(?=\\s*\\()',                       // function/method calls
  ].join('|'), 'g');

  return safe.replace(TOKEN_RE, (match, blockCmt, lineCmt, tmpl, dStr, sStr, kw, num, fn) => {
    if (blockCmt || lineCmt) return `<span class="hl-comment">${match}</span>`;
    if (tmpl || dStr || sStr) return `<span class="hl-string">${match}</span>`;
    if (kw)                   return `<span class="hl-keyword">${match}</span>`;
    if (num)                  return `<span class="hl-number">${match}</span>`;
    if (fn)                   return `<span class="hl-fn">${match}</span>`;
    return match;
  });
}

function buildStatsHTML(info) {
  const rows = [
    ['Year',             info.year             ?? '—'],
    ['Time Complexity',  info.timeComplexity   || '—'],
    ['Space Complexity', info.spaceComplexity  || '—'],
    ['Optimal',          info.optimal          || '—'],
    ['Complete',         info.complete         || '—'],
  ];
  if (info.graphType         !== undefined) rows.push(['Graph Type',        info.graphType        || '—']);
  if (info.heuristic         !== undefined) rows.push(['Heuristic',         info.heuristic        || 'None']);
  if (info.samplingStrategy  !== undefined) rows.push(['Sampling Strategy', info.samplingStrategy || '—']);
  if (info.queryType         !== undefined) rows.push(['Query Type',        info.queryType        || '—']);
  if (info.predecessor)                     rows.push(['Predecessor',       info.predecessor]);
  if (info.successor)                       rows.push(['Successor',         info.successor]);

  const statsRows = rows.map(([label, value]) =>
    `<div class="stat-row"><span class="stat-label">${label}</span><span class="stat-value">${value}</span></div>`
  ).join('');

  const improvement = info.notableImprovement
    ? `<div class="stat-improvement"><span class="stat-label">Notable Improvement</span><p>${info.notableImprovement}</p></div>`
    : '';

  const goodFor = (info.goodFor || []).map(s => `<span class="stat-badge stat-badge-good">${s}</span>`).join('');
  const badFor  = (info.badFor  || []).map(s => `<span class="stat-badge stat-badge-bad">${s}</span>`).join('');

  return `<div class="stat-grid">${statsRows}</div>${improvement}` +
    (goodFor ? `<div class="stat-section-label">Good for</div><div class="stat-badges">${goodFor}</div>` : '') +
    (badFor  ? `<div class="stat-section-label">Avoid when</div><div class="stat-badges">${badFor}</div>` : '');
}

function updateInfoPanel(info, impl) {
  if (!info) return;
  document.getElementById('info-name').textContent        = info.displayName;
  document.getElementById('info-description').textContent = info.description;

  const paperLink = document.getElementById('info-paper');
  if (info.paperUrl) {
    paperLink.href        = info.paperUrl;
    paperLink.textContent = (info.paperTitle || 'Read Paper') + ' \u2192';
    paperLink.classList.remove('hidden');
  } else {
    paperLink.classList.add('hidden');
  }

  document.getElementById('pseudocode-content').textContent = info.pseudocode;
  const header = `// ${info.displayName} — run() implementation\n\n`;
  document.getElementById('implementation-content').innerHTML = highlightJS(header + impl.run.toString());
  document.getElementById('stats-content').innerHTML = buildStatsHTML(info);
}

// ============================================================
// DOCUMENT-LEVEL MOUSE EVENTS
// ============================================================

document.addEventListener('mousemove', e => {
  if (!state.isMouseDownCanvas) return;
  const rect = canvasEl.getBoundingClientRect();
  const x    = e.clientX - rect.left;
  const y    = e.clientY - rect.top;

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

  // Canvas – finalise obstacle
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


btnClearBoard.addEventListener('click', () => {
  cancelRun();
  if (state.algorithmType === 'grid') clearGridBoard();
  else                                clearCanvasBoard();
  setStatus('Board cleared.');
});

// Changing algorithm type cancels the run and clears the board
algorithmTypeSel.addEventListener('change', () => {
  cancelRun();
  if (state.algorithmType === 'grid') clearGridBoard();
  else                                clearCanvasBoard();
  switchAlgorithmType(algorithmTypeSel.value);
});

// Changing algorithm cancels the run and clears the path
algorithmSel.addEventListener('change', () => {
  cancelRun();
  if (state.algorithmType === 'grid') clearGridPath();
  else                                clearCanvasPath();

  const entry = Algorithm.getById(algorithmSel.value);
  if (entry) updateInfoPanel(entry.info, entry.impl);
});

function switchToTab(name) {
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(c => c.classList.add('hidden'));
  const btn = document.querySelector(`.tab-btn[data-tab="${name}"]`);
  if (btn) btn.classList.add('active');
  const panel = document.getElementById(`tab-${name}`);
  if (panel) panel.classList.remove('hidden');
  if (name === 'visualizer') resizeCanvas();
}

document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => switchToTab(btn.dataset.tab));
});

// ============================================================
// BOOTSTRAP
// ============================================================
initGrid();
initCanvas();
switchAlgorithmType('sampling');
switchToTab('visualizer');
window.addEventListener('resize', resizeCanvas);
