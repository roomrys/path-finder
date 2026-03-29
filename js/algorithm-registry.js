'use strict';

/* ============================================================
   js/algorithm-registry.js

   AlgorithmInfo  — pure metadata base classes (no run logic).
   Algorithm      — coordinator / registry that ties each
                    AlgorithmInfo subclass to its AlgorithmImpl.

   Load order:  algorithm-registry.js
             -> algorithm-impls.js
             -> grid-algorithms.js | sampling-algorithms.js
             -> main.js
   ============================================================ */

// ---------------------------------------------------------------------------
// AlgorithmInfo – base metadata class
//   Subclassed per algorithm type; never holds run() logic.
// ---------------------------------------------------------------------------
class AlgorithmInfo {
  static id          = '';
  static displayName = '';
  static type        = '';   // 'grid' | 'sampling'
  static description = '';
  static paperUrl    = null; // optional URL to the original paper
  static paperTitle  = null; // link text for paperUrl
  static pseudocode  = '';

  // ── Stats (applies to both grid and sampling algorithms) ─────────────────
  static year               = null;  // number — year first published
  static timeComplexity     = '';    // e.g. 'O(V + E)'
  static spaceComplexity    = '';    // e.g. 'O(V)'
  // 'Yes [— reason]' | 'Asymptotically optimal' | 'No — <reason>'
  static optimal            = '';
  // 'Yes' | 'Probabilistically complete' | 'No — <reason>'
  static complete           = '';
  static predecessor        = null;  // string | null — algorithm this extends
  static notableImprovement = null;  // string | null — key insight vs predecessor
  static successor          = null;  // string | null — algorithms that build on this
  static goodFor            = [];    // string[]
  static badFor             = [];    // string[]
}

// ---------------------------------------------------------------------------
// GridAlgorithmInfo – metadata for grid-based algorithms
// ---------------------------------------------------------------------------
class GridAlgorithmInfo extends AlgorithmInfo {
  static type = 'grid';

  // 'unweighted' | 'weighted' — whether the algorithm uses edge costs
  static graphType = '';
  // null | string — heuristic used (e.g. 'Manhattan distance')
  static heuristic = null;
}

// ---------------------------------------------------------------------------
// SamplingAlgorithmInfo – metadata for sampling-based algorithms
// ---------------------------------------------------------------------------
class SamplingAlgorithmInfo extends AlgorithmInfo {
  static type = 'sampling';

  // 'tree' | 'roadmap' | 'batch' — core exploration data structure
  static samplingStrategy = '';
  // 'single' | 'multi' — whether the structure supports reuse across queries
  static queryType = '';
}

// ---------------------------------------------------------------------------
// Algorithm – coordinator / registry
//
//   Algorithm.register(InfoClass, ImplClass)
//     Links a metadata class to its implementation class by shared id.
//
//   Algorithm.getById(id)        → { info, impl } | null
//   Algorithm.getAllOfType(type) → [{ info, impl }, ...]  (insertion order)
// ---------------------------------------------------------------------------
class Algorithm {
  static _registry = new Map(); // id -> { info: AlgorithmInfo, impl: AlgorithmImpl }

  static register(infoClass, implClass) {
    Algorithm._registry.set(infoClass.id, { info: infoClass, impl: implClass });
  }

  static getById(id) {
    return Algorithm._registry.get(id) || null;
  }

  static getAllOfType(type) {
    const out = [];
    for (const entry of Algorithm._registry.values()) {
      if (entry.info.type === type) out.push(entry);
    }
    return out;
  }
}
