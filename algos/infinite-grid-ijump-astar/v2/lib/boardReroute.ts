// Board-level selection + net-ordering helpers used to remove "wild trace
// jumps" (https://github.com/tscircuit/autorouting/issues/92).
//
// A wild jump (a route that shoots far outside its endpoints' bounding box and
// comes back) is caused by the greedy router committing an early net that boxes
// a later net into a detour — it is a NET-ORDER problem, not a per-net search
// problem. When a board routes wild, we re-route the whole board under
// alternative net orderings (shortest-first, then a few deterministic shuffles)
// and keep the least-wild board that does not lose nets or break clearance.

import type { SimpleRouteConnection } from "solver-utils"
import { manDist } from "./util"
import {
  type ConnectionSolveResult,
  type PointWithLayer,
  getEndpointBBoxExcursion,
} from "./GeneralizedAstar"

export function getRouteLength(route: PointWithLayer[]): number {
  let len = 0
  for (let i = 1; i < route.length; i++) len += manDist(route[i - 1], route[i])
  return len
}

export interface BoardMetrics {
  routedCount: number
  wildNetCount: number
  totalWildExcess: number
  maxExcursion: number
  totalExcursion: number
  totalWireLength: number
}

export function getBoardMetrics(
  solutions: ConnectionSolveResult[],
  threshold: number,
): BoardMetrics {
  let routedCount = 0
  let wildNetCount = 0
  let totalWildExcess = 0
  let maxExcursion = 0
  let totalExcursion = 0
  let totalWireLength = 0
  for (const s of solutions) {
    if (!s.solved) continue
    routedCount++
    const e = getEndpointBBoxExcursion(s.route)
    const wildExcess = Math.max(0, e - threshold)
    if (wildExcess > 0) wildNetCount++
    totalWildExcess += wildExcess
    totalExcursion += e
    maxExcursion = Math.max(maxExcursion, e)
    totalWireLength += getRouteLength(s.route)
  }
  return {
    routedCount,
    wildNetCount,
    totalWildExcess,
    maxExcursion,
    totalExcursion,
    totalWireLength,
  }
}

/**
 * Lower is better. Severity of wildness first (so one egregious 70mm detour is
 * worse than two mild 18mm ones), then how wild the worst net is, then the
 * count of wild nets, then completeness, then wirelength.
 */
export function compareBoardMetrics(
  a: BoardMetrics,
  b: BoardMetrics,
  GRID_STEP: number,
): number {
  if (Math.abs(a.totalWildExcess - b.totalWildExcess) > GRID_STEP)
    return a.totalWildExcess - b.totalWildExcess
  if (Math.abs(a.maxExcursion - b.maxExcursion) > GRID_STEP)
    return a.maxExcursion - b.maxExcursion
  if (a.wildNetCount !== b.wildNetCount) return a.wildNetCount - b.wildNetCount
  if (Math.abs(a.totalExcursion - b.totalExcursion) > GRID_STEP)
    return a.totalExcursion - b.totalExcursion
  if (a.routedCount !== b.routedCount) return b.routedCount - a.routedCount
  if (Math.abs(a.totalWireLength - b.totalWireLength) > GRID_STEP)
    return a.totalWireLength - b.totalWireLength
  return 0
}

// --- net ordering --------------------------------------------------------

function connectionSpan(c: SimpleRouteConnection): number {
  const pts = c.pointsToConnect
  if (pts.length < 2) return 0
  const a = pts[0]
  const b = pts[pts.length - 1]
  return Math.abs(a.x - b.x) + Math.abs(a.y - b.y)
}

/** Route the shortest connections first — long early nets are what box later nets in. */
export function sortConnectionsShortestFirst(
  connections: SimpleRouteConnection[],
): SimpleRouteConnection[] {
  return [...connections].sort((a, b) => connectionSpan(a) - connectionSpan(b))
}

function mulberry32(seed: number): () => number {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6d2b79f5) | 0
    let t = Math.imul(a ^ (a >>> 15), 1 | a)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

/** Deterministic Fisher-Yates shuffle (fixed seed → reproducible router output). */
export function seededShuffleConnections(
  connections: SimpleRouteConnection[],
  seed: number,
): SimpleRouteConnection[] {
  const rnd = mulberry32(seed)
  const a = [...connections]
  for (let i = a.length - 1; i > 0; i--) {
    const j = Math.floor(rnd() * (i + 1))
    ;[a[i], a[j]] = [a[j], a[i]]
  }
  return a
}

// --- DRC-by-construction guard -------------------------------------------

interface ObstacleLike {
  center: { x: number; y: number }
  width: number
  height: number
  layers: string[]
  connectedTo: string[]
}

/** Liang-Barsky: does segment a→b touch the axis-aligned rect [minX,maxX]×[minY,maxY]? */
function segmentIntersectsRect(
  a: { x: number; y: number },
  b: { x: number; y: number },
  minX: number,
  maxX: number,
  minY: number,
  maxY: number,
): boolean {
  let t0 = 0
  let t1 = 1
  const dx = b.x - a.x
  const dy = b.y - a.y
  const clip = (p: number, q: number): boolean => {
    if (p === 0) return q >= 0
    const r = q / p
    if (p < 0) {
      if (r > t1) return false
      if (r > t0) t0 = r
    } else {
      if (r < t0) return false
      if (r < t1) t1 = r
    }
    return true
  }
  if (
    clip(-dx, a.x - minX) &&
    clip(dx, maxX - a.x) &&
    clip(-dy, a.y - minY) &&
    clip(dy, maxY - a.y)
  ) {
    return t0 <= t1
  }
  return false
}

/**
 * Count how many board routes pass copper over an obstacle (pad/keepout) that
 * belongs to a different net — i.e. real cross-net DRC overlaps. A candidate
 * board is only acceptable if this is zero, so the selected board is DRC-clean
 * by construction. `getNetId` maps a connection name to its connectivity-net id
 * (so a route may legally pass over its own net's pads).
 */
export function countCrossNetObstacleOverlaps(
  solutions: ConnectionSolveResult[],
  obstacles: ObstacleLike[],
  getNetId: (connectionName: string) => string,
  traceWidth: number,
): number {
  const halfW = traceWidth / 2
  let overlaps = 0
  for (const s of solutions) {
    if (!s.solved) continue
    const net = getNetId(s.connectionName)
    for (let i = 1; i < s.route.length; i++) {
      const a = s.route[i - 1]
      const b = s.route[i]
      if (a.layer !== b.layer) continue // via transition, not a copper segment
      for (const o of obstacles) {
        if (o.connectedTo.includes(net)) continue // same net: allowed contact
        if (!o.layers.includes(a.layer)) continue
        const minX = o.center.x - o.width / 2 - halfW
        const maxX = o.center.x + o.width / 2 + halfW
        const minY = o.center.y - o.height / 2 - halfW
        const maxY = o.center.y + o.height / 2 + halfW
        if (segmentIntersectsRect(a, b, minX, maxX, minY, maxY)) {
          overlaps++
          break
        }
      }
    }
  }
  return overlaps
}
