import { getKeyboardGenerator } from "autorouting-dataset/lib/generators/keyboards"
import { test, expect } from "bun:test"
import { getFullConnectivityMapFromCircuitJson } from "circuit-json-to-connectivity-map"
import { getSimpleRouteJson } from "solver-utils"
import { checkEachPcbTraceNonOverlapping } from "@tscircuit/checks"
import { MultilayerIjump } from "algos/multi-layer-ijump/MultilayerIjump"
import { getEndpointBBoxExcursion } from "algos/infinite-grid-ijump-astar/v2/lib/GeneralizedAstar"

// Reproduction for https://github.com/tscircuit/autorouting/issues/92
// ("Multilayer Ijump: Remove wild trace jumps").
//
// Routing the nets in the input order, the greedy router commits an early net
// that boxes a later net into a gross out-and-back detour — on keyboard
// generator seed 27 a net whose two pads are ~10mm apart routes via a ~75mm
// excursion and back. The board reroute (enabled by default) re-routes the
// whole board under alternative net orderings (shortest-first, then
// deterministic shuffles) and keeps the least-wild board that does not lose
// routed nets or introduce cross-net trace overlaps, removing the wild jump.

test("repro4: getEndpointBBoxExcursion measures out-and-back excursion", () => {
  // A monotonic run toward the goal stays inside the endpoint bbox → 0.
  expect(
    getEndpointBBoxExcursion([
      { x: 0, y: 0, layer: "top" },
      { x: 25, y: 0, layer: "top" },
      { x: 50, y: 0, layer: "top" },
    ]),
  ).toBe(0)
  // A path that shoots 75mm away from both endpoints and back → 75.
  expect(
    getEndpointBBoxExcursion([
      { x: 0, y: 0, layer: "top" },
      { x: -75, y: 0, layer: "top" },
      { x: 10, y: 0, layer: "top" },
    ]),
  ).toBeCloseTo(75)
})

function routeSeed27(isWildJumpRerouteEnabled: boolean) {
  return getKeyboardGenerator()
    .getExample({ seed: 27 })
    .then((soup: any) => {
      const connMap = getFullConnectivityMapFromCircuitJson(soup)
      const input = getSimpleRouteJson(soup, { layerCount: 2, connMap })
      const autorouter = new MultilayerIjump({
        input,
        connMap,
        optimizeWithGoalBoxes: true,
        isWildJumpRerouteEnabled,
      })
      const traces = autorouter.solveAndMapToTraces() as any[]
      let maxExcursion = 0
      let routedCount = 0
      for (const t of traces) {
        if (!t.route || t.route.length < 2) continue
        routedCount++
        maxExcursion = Math.max(
          maxExcursion,
          getEndpointBBoxExcursion(
            t.route.map((p: any) => ({ x: p.x, y: p.y, layer: p.layer })),
          ),
        )
      }
      const overlapErrors = checkEachPcbTraceNonOverlapping(
        soup.concat(traces),
      ).length
      return { maxExcursion, routedCount, overlapErrors }
    })
}

test("repro4: net-order reroute removes the seed-27 wild jump", async () => {
  const greedy = await routeSeed27(false)
  const fixed = await routeSeed27(true)

  // In input order the greedy router commits a gross out-and-back: the worst
  // route leaves its endpoints' bounding box by >60mm.
  expect(greedy.maxExcursion).toBeGreaterThan(60)

  // After the reroute the board is no longer wild — the worst excursion is
  // below the wild threshold (10mm).
  expect(fixed.maxExcursion).toBeLessThan(10)

  // ...without losing routed nets or introducing trace overlaps.
  expect(fixed.routedCount).toBeGreaterThanOrEqual(greedy.routedCount)
  expect(fixed.overlapErrors).toBe(0)
})
