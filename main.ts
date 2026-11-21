// Sokoban solver for make.fun's sokoban-solver-01-run-20261121.
// A push-optimal solver: each level is split into its independent rooms
// (a box pushed out of a room's door can never come back, so rooms add up),
// and every room is solved with A* over pushes — a state is the boxes plus
// the player's reachable region — with an admissible Hungarian heuristic
// (minimum-push distances per box, pulled back from the goals ignoring the
// other boxes), dead-square pruning and frozen 2x2-block pruning. The
// search is budgeted by work (state counts), never by the clock, so two
// runs write the same bytes. A plain push-BFS (same code, heuristic off)
// is the fallback when A* exhausts its budget.
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets only (never the clock, so runs are deterministic).
const ASTAR_STATES = 300_000;
const BFS_STATES = 300_000;
const TOTAL_STATES = 2_500_000;
let spent = 0;

const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  size: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, size: w * h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/**
 * Minimum pushes for one box to reach each goal, ignoring the other boxes:
 * a pull-back BFS from the goals (a box at x came from y = x - d when the
 * player could stand at y - d). dist[g * size + x] is that minimum, or -1.
 * A square with -1 for every goal is dead: no box there can ever be solved.
 */
function pushDistances(level: Level): Int32Array {
  const { w, wall, goals, size } = level;
  const dist = new Int32Array(goals.length * size).fill(-1);
  for (let gi = 0; gi < goals.length; gi++) {
    const base = gi * size;
    const g = goals[gi];
    const queue: number[] = [g];
    dist[base + g] = 0;
    for (let qi = 0; qi < queue.length; qi++) {
      const x = queue[qi];
      const d0 = dist[base + x];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const y = x - d;
        if (y < 0 || y >= size || wall[y] || wall[y - d] || dist[base + y] >= 0) continue;
        dist[base + y] = d0 + 1;
        queue.push(y);
      }
    }
  }
  return dist;
}

function liveOf(level: Level, dist: Int32Array): Uint8Array {
  const live = new Uint8Array(level.size);
  for (let x = 0; x < level.size; x++) {
    for (let gi = 0; gi < level.goals.length; gi++) {
      if (dist[gi * level.size + x] >= 0) {
        live[x] = 1;
        break;
      }
    }
  }
  return live;
}

/** Lowest-cost assignment of boxes to goals (brute force, at most 4 boxes). */
function hungarian(boxes: number[], goals: number[], dist: Int32Array, size: number): number {
  const n = boxes.length;
  if (n === 0) return 0;
  const cost: number[][] = [];
  for (let b = 0; b < n; b++) {
    cost.push([]);
    for (let g = 0; g < goals.length; g++) {
      const d = dist[g * size + boxes[b]];
      cost[b].push(d < 0 ? 1e9 : d);
    }
  }
  // All permutations of goal indices (n <= 4, so at most 24).
  const perm: number[] = [];
  for (let g = 0; g < goals.length; g++) perm.push(g);
  let best = 1e9;
  const rec = (i: number, used: boolean[], sum: number): void => {
    if (sum >= best) return;
    if (i === n) {
      best = sum;
      return;
    }
    for (let g = 0; g < goals.length; g++) {
      if (used[g]) continue;
      used[g] = true;
      rec(i + 1, used, sum + cost[i][g]);
      used[g] = false;
    }
  };
  rec(0, new Array(goals.length).fill(false), 0);
  return best;
}

/** The player's reachable region from `from` with `occupied` in the way; its lowest square. */
function regionLow(
  level: Level,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  const { w, wall, size } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    const q0 = p - w;
    const q1 = p + w;
    const q2 = p - 1;
    const q3 = p + 1;
    if (q0 >= 0 && wall[q0] === 0 && occupied[q0] === 0 && seen[q0] !== stamp) {
      seen[q0] = stamp;
      stack[n++] = q0;
    }
    if (q1 < size && wall[q1] === 0 && occupied[q1] === 0 && seen[q1] !== stamp) {
      seen[q1] = stamp;
      stack[n++] = q1;
    }
    if (wall[q2] === 0 && occupied[q2] === 0 && seen[q2] !== stamp) {
      seen[q2] = stamp;
      stack[n++] = q2;
    }
    if (wall[q3] === 0 && occupied[q3] === 0 && seen[q3] !== stamp) {
      seen[q3] = stamp;
      stack[n++] = q3;
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes with a box off its goal. */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (a < 0 || d >= level.size) continue;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** Shortest walk from `from` to `to` around walls and boxes, in lower LURD letters (u first on ties). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string | null {
  const { w, wall, size } = level;
  if (from === to) return '';
  const prev = new Int32Array(size).fill(-1);
  const how = new Array<string>(size);
  const queue: number[] = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (q < 0 || q >= size || wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return null;
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

interface Solved {
  pushes: number;
  moves: string;
  endPlayer: number;
}

/**
 * Push-optimal search over one level (a room, or a whole level as fallback).
 * A state is the boxes plus the player's region (named by its lowest square);
 * expanding a state tries every push the player can reach. `heuristic` picks
 * A* (Hungarian lower bound) or plain BFS (heuristic 0). Deterministic:
 * fixed direction order, heap ties by deeper g then by insertion order.
 */
function searchPush(level: Level, dist: Int32Array, live: Uint8Array, maxStates: number, heuristic: boolean): Solved | null {
  const { w, wall, goals, size } = level;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (level.boxes.some((b) => !live[b])) return null;
  if (solved(level.boxes)) return { pushes: 0, moves: '', endPlayer: level.player };

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const keyOf = (boxes: number[], low: number): number => {
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };

  const boxesArr: number[][] = [level.boxes.slice()];
  const playerArr: number[] = [level.player];
  const lowArr: number[] = [0];
  const gArr: number[] = [0];
  const parent: number[] = [-1];
  const pushFrom: number[] = [-1];
  const pushTo: number[] = [-1];
  const pushDir: number[] = [-1];
  const best = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  lowArr[0] = regionLow(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  best.set(keyOf(level.boxes, lowArr[0]), 0);

  // Binary heap of [f, -g, seq, state]. Smaller f first, deeper g first, FIFO.
  const heap: [number, number, number, number][] = [];
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const tmp = heap[i];
      heap[i] = heap[p];
      heap[p] = tmp;
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const tmp = heap[i];
        heap[i] = heap[c];
        heap[c] = tmp;
        i = c;
      }
    }
    return top;
  };

  const h0 = heuristic ? hungarian(level.boxes, goals, dist, size) : 0;
  if (h0 >= 1e9) return null;
  let seq = 0;
  heapPush([h0, 0, seq++, 0]);
  let found = -1;
  let expanded = 0;

  while (heap.length > 0) {
    const [, negG, , s] = heapPop();
    const g = -negG;
    // Stale entry: this state was re-reached with fewer pushes.
    if (best.get(keyOf(boxesArr[s], lowArr[s])) !== s) continue;
    const boxes = boxesArr[s];
    if (solved(boxes)) {
      found = s;
      break;
    }
    if (expanded >= maxStates || spent >= TOTAL_STATES) return null;
    expanded++;
    spent++;
    for (const b of boxes) occupied[b] = 1;
    const here = ++stamp;
    regionLow(level, occupied, playerArr[s], seen, here, stack);
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi];
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di];
        const d = dy * w + dx;
        const t = b + d;
        if (t < 0 || t >= size || wall[t] || occupied[t] || !live[t]) continue;
        if (seen[b - d] !== here) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = -1;
        let low = -1;
        let next: number[] | null = null;
        let h = 0;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((a, b2) => a - b2);
          low = regionLow(level, occupied, b, mark, ++stamp, stack);
          key = keyOf(next, low);
          if (heuristic) h = hungarian(next, goals, dist, size);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || h >= 1e9) continue;
        const prevBest = best.get(key);
        if (prevBest !== undefined && gArr[prevBest] <= g + 1) continue;
        const ns = boxesArr.length;
        best.set(key, ns);
        boxesArr.push(next!);
        playerArr.push(b);
        lowArr.push(low);
        gArr.push(g + 1);
        parent.push(s);
        pushFrom.push(b);
        pushTo.push(t);
        pushDir.push(di);
        if (solved(next!)) {
          found = ns;
          break;
        }
        heapPush([g + 1 + h, -(g + 1), seq++, ns]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
    if (found >= 0) break;
  }
  if (found < 0) return null;

  // Rebuild the moves: walk to each push's square, then push.
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const nextBoxes = boxesArr[s];
    const from = pushFrom[s];
    const to = pushTo[s];
    const di = pushDir[s];
    const d = DIRS[di][1] * w + DIRS[di][0];
    if (to - from !== d) throw new Error('internal: push direction mismatch');
    for (const b of boxes) occupied[b] = 1;
    const walk = walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    if (walk === null) return null;
    moves += walk + DIRS[di][3];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves, endPlayer: player };
}

interface Room {
  boxes: number[];
  goals: number[];
  done: boolean;
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway:
 * a box there reaches no goal) out of the floor; each part left holding a
 * box is a room. Null when the level is no set of rooms.
 */
function roomsOf(level: Level, live: Uint8Array): Room[] | null {
  const { w, wall, goals, boxes, player, size } = level;
  const row = Math.floor(player / w);
  const cut = new Uint8Array(size);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(size).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < size; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares: number[] = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i] + dy * w + dx;
        if (q < 0 || q >= size || wall[q] || cut[q] || part[q] >= 0) continue;
        part[q] = rooms.length;
        squares.push(q);
      }
    }
    rooms.push({ boxes: [], goals: [], done: false });
  }
  for (const b of boxes) {
    if (part[b] < 0) return null;
    rooms[part[b]].boxes.push(b);
  }
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]].goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Solve one level room by room (or whole-level when it is no set of rooms). */
function solveLevel(level: Level): Solved | null {
  const dist = pushDistances(level);
  const live = liveOf(level, dist);
  const rooms = roomsOf(level, live);
  if (rooms === null) {
    return searchPush(level, dist, live, BFS_STATES, false) ?? searchPush(level, dist, live, BFS_STATES, true);
  }
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    wall[player] = 0; // the player stands on a walkable square, even on a done room's goal
    const sub: Level = {
      w: level.w,
      h: level.h,
      size: level.size,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const subDist = pushDistances(sub);
    const subLive = liveOf(sub, subDist);
    const solved =
      searchPush(sub, subDist, subLive, ASTAR_STATES, true) ??
      searchPush(sub, subDist, subLive, BFS_STATES, false);
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = solved.endPlayer;
    room.done = true;
  }
  return { pushes, moves, endPlayer: player };
}

function solve(text: string): string {
  try {
    const level = parseLevel(text);
    if (level.player < 0 || level.boxes.length === 0) return '';
    const solved = solveLevel(level);
    return solved === null ? '' : solved.moves;
  } catch {
    return '';
  }
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
