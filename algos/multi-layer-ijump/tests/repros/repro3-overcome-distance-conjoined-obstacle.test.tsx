import { test, expect } from "bun:test"
import { ObstacleList3d } from "algos/multi-layer-ijump/ObstacleList3d"
import { getDistanceToOvercomeObstacle } from "algos/infinite-grid-ijump-astar/v2/lib/getDistanceToOvercomeObstacle"

// Reproduction for https://github.com/tscircuit/autorouting/issues/92
// ("Multilayer Ijump: Remove wild trace jumps").
//
// A wild trace jump happens when the autorouter under-estimates how far it has
// to travel to get past an obstacle: it thinks it has cleared the obstacle,
// turns, immediately runs back into it, and is forced to switch direction.
//
// `getDistanceToOvercomeObstacle` is where that distance is computed. When a
// second ("conjoined") obstacle sits right after the one we hit, the old code
// only sampled a SINGLE point at the end of the first obstacle to look for the
// next one. If that point happened to miss the conjoined obstacle (because the
// obstacle is shorter and the sample point sits just above/beside it) the
// returned distance only cleared the first obstacle, leaving the trace blocked.
//
// This test builds exactly that geometry and asserts the returned distance is
// large enough to clear BOTH obstacles. Against the old single-point
// implementation the returned distance is 1.3 (it stops at the first obstacle's
// edge), which fails the assertion below.
test("repro3: overcome distance accounts for conjoined obstacle", () => {
  const OBSTACLE_MARGIN = 0.15

  // o1: the tall obstacle the trace hits, x[-0.5, 0.5], y[-1.5, 1.5]
  // o2: a SHORTER conjoined obstacle continuing further along +x,
  //     x[0.5, 2.5], y[-1.5, -0.3] — its top edge (-0.3) sits well below the
  //     point the old code sampled, so the single-point check missed it.
  const obstacles = new ObstacleList3d(2, [
    {
      type: "rect",
      center: { x: 0, y: 0 },
      width: 1,
      height: 3,
      layers: ["top", "bottom"],
      connectedTo: [],
    } as any,
    {
      type: "rect",
      center: { x: 1.5, y: -0.9 },
      width: 2,
      height: 1.2,
      layers: ["top", "bottom"],
      connectedTo: [],
    } as any,
  ])

  const obstacle = obstacles.obstacles.find(
    (o) => o.center.x === 0 && o.l === 0,
  )!

  // Trace travels +x, hugging the bottom wall of o1 (wallDir points +y, the
  // OBSTACLE_MARGIN-thick band it would sweep if it turned).
  const node = { x: -0.5 - OBSTACLE_MARGIN, y: -1.5 - OBSTACLE_MARGIN, l: 0 }

  const distance = getDistanceToOvercomeObstacle({
    node,
    travelDir: { dx: 1, dy: 0, wallDistance: 10 } as any,
    wallDir: { dx: 0, dy: 1, wallDistance: OBSTACLE_MARGIN } as any,
    obstacle: obstacle as any,
    obstacles: obstacles as any,
    OBSTACLE_MARGIN,
    SHOULD_DETECT_CONJOINED_OBSTACLES: true,
  })

  const endX = node.x + distance

  // o2's far (right) edge is at x = 2.5; clearing it requires endX >= 2.5 + margin.
  const o2FarEdgeWithMargin = 2.5 + OBSTACLE_MARGIN
  expect(endX).toBeGreaterThanOrEqual(o2FarEdgeWithMargin)
})
