/*
 * solver.c - Team T13 - robot name: ????          <- TODO: your robot's name
 *
 * ENCS4380 Micromouse maze solver, HAL "ENCS4380-2026/27-S1".
 *
 * Rules this file follows (mouse_hal.h and the kit README):
 *   - includes only mouse_hal.h and standard C headers,
 *   - talks to the mouse only through mm_... functions,
 *   - never prints with printf(); every message goes through mm_log(),
 *   - reads the maze size and goal from mm_maze(); nothing is hard-coded,
 *   - stores walls with MM_WALL(dir) only (the bit values change every semester),
 *   - builds unchanged for the simulator (hal_sim.c) and the robot (hal_esp32.cpp).
 *
 * How it works (each part is explained again where it is written):
 *
 *   1. Map. Two bit maps per cell, both in MM_WALL() bits:
 *        walls[x][y]  sides we believe are walls (the outer wall is set from the start),
 *        known[x][y]  sides we have actually observed (sensed, or driven through).
 *
 *   2. Pose. x, y and heading. The HAL does not track them, so we update them
 *      after every mm_forward() and mm_turn() that returns MM_OK.
 *
 *   3. Sensing. mm_sense() is RELATIVE (left, front, right). With heading h,
 *      left = h+3, front = h, right = h+1 (mod 4). Each wall is shared by two
 *      cells, so we always write both sides.
 *
 *   4. Flood fill. A breadth-first search with a queue, started from the target
 *      cells. dist[c] = fewest cells from c to the nearest target. The mouse
 *      always steps to the open neighbour with the smallest dist.
 *        - Exploring: a side we have not seen is treated as OPEN (optimistic).
 *          If the guess is wrong we see the wall, re-flood, and go another way.
 *          This is also why dead ends are left automatically: once their walls
 *          are known their distance grows and the mouse turns back.
 *        - Speed run: only sides we have SEEN to be open are used, so the path
 *          is guaranteed to exist.
 *
 *   5. Phases.
 *        a) explore from the start to the goal,
 *        b) explore back to the start (often finds a better route),
 *        c) PROVE: keep visiting cells until the best known path is as short as
 *           the best path that assumes every unseen wall is open; then no
 *           shorter path can exist (optional, PROVE_SHORTEST_PATH),
 *        d) return to the start on known walls,
 *        e) speed run on known walls (fewest cells, then fewest turns),
 *        f) return to the start and report the map.
 *      The map is reported with mm_map_begin/cell/end after every arrival at
 *      the goal and once more at the end.
 *
 *   6. Reset. Before every sense and every move we ask mm_reset_requested().
 *      On a reset the pose goes back to (0,0) facing north, the map is KEPT,
 *      mm_reset_done() is called, and the current phase carries on from there.
 *
 *   7. Robot safety net (never triggers in the simulator, whose sensors are
 *      perfect). Sides proven by motion (driven through, or a blocked move)
 *      are "sure" and a sensor reading cannot overwrite them. If a multi-cell
 *      move is blocked, the mouse no longer knows where it is: it reports the
 *      map, waits for the operator to reset it, and from then on crosses
 *      unproven sides one cell at a time so the next bump locates the wall.
 *      If a wrong "wall" reading cuts the mouse off from its target, the
 *      walls on that cut that only the sensors reported are set back to
 *      unknown, and the mouse tries them by driving (see reopen_cut_walls).
 */
#include "mouse_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ==== Settings you may change =========================================== */

#define TEAM_ID              "T13"  /* team number, the same as TEAM_ID in robot_config.h */
#define PROVE_SHORTEST_PATH  1      /* 1 = explore until the shortest path is certain,
                                       0 = speed-run the best path found on the way back */
#define SPEED_RUNS           1      /* speed runs after exploring (each one returns to start) */
#define SHOW_DISTANCES       1      /* 1 = write flood distances on the maze (simulator) */
#define REPORT_PERIMETER_AS_KNOWN 0 /* 1 = report the outer wall of every cell as known.
                                       0 = report only sides we really saw (the README rule:
                                       "a side you never saw should be reported as unknown") */

/* Speed-run planner cost: cells * SPEED_CELL_COST + quarter turns * SPEED_TURN_COST.
 * With SPEED_CELL_COST far above any possible turn count, the planner picks the
 * path with the fewest cells, and among those the one with the fewest turns.
 * On the robot you may lower SPEED_CELL_COST (for example 10 against a turn cost
 * of 15) if your turns are slow and you prefer a slightly longer, straighter path. */
#define SPEED_CELL_COST      1024u
#define SPEED_TURN_COST      1u

/* ==== Types ============================================================== */

typedef enum { GO_OK, GO_RESET, GO_BLOCKED, GO_FAULT } go_t;         /* one command   */
typedef enum { RUN_DONE, RUN_RESET, RUN_NO_PATH, RUN_FAULT } run_t;  /* one phase leg */
typedef enum { TARGET_GOAL, TARGET_START, TARGET_PROOF } target_t;
typedef enum { PLAN_OPTIMISTIC, PLAN_KNOWN } plan_t;

#define N_CELLS   (MM_MAX_SIZE * MM_MAX_SIZE)
#define DIST_INF  0xFFFFu

/* ==== State ============================================================== */

static int g_w, g_h;                          /* maze size from mm_maze()        */
static int g_gx0, g_gy0, g_gx1, g_gy1;        /* goal block, inclusive           */

static uint8_t  g_walls[MM_MAX_SIZE][MM_MAX_SIZE];    /* [x][y], MM_WALL() bits  */
static uint8_t  g_known[MM_MAX_SIZE][MM_MAX_SIZE];    /* [x][y], MM_WALL() bits  */
static uint8_t  g_sure[MM_MAX_SIZE][MM_MAX_SIZE];     /* sides proven by motion:
                                                         driven through or bumped */
static bool     g_cautious;   /* set after a multi-cell move was blocked: from then on
                                 only sides in g_sure are merged into one mm_forward(n) */
static bool     g_visited[MM_MAX_SIZE][MM_MAX_SIZE];  /* sensed from inside      */
static bool     g_target[MM_MAX_SIZE][MM_MAX_SIZE];   /* cells we are heading to */
static uint16_t g_dist[MM_MAX_SIZE][MM_MAX_SIZE];     /* flood distances         */

static int g_x, g_y, g_head;                  /* pose: cell and heading          */
static int g_resets;                          /* number of operator resets       */
static int g_reopens;                         /* times walls were re-opened after
                                                 a wrong reading cut off the target */
#define MAX_REOPENS 16                        /* give up after this many         */

/* ==== Directions ========================================================= */
/* mouse_hal.h: N=0, E=1, S=2, W=3, and turning right adds 1 (mod 4). */

static int dir_dx(int d)     { return d == MM_EAST ? 1 : (d == MM_WEST ? -1 : 0); }
static int dir_dy(int d)     { return d == MM_NORTH ? 1 : (d == MM_SOUTH ? -1 : 0); }
static int dir_right(int d)  { return (d + 1) % 4; }
static int dir_left(int d)   { return (d + 3) % 4; }
static int dir_back(int d)   { return (d + 2) % 4; }

static char dir_name(int d)
{
    switch (d) {
    case MM_NORTH: return 'N';
    case MM_EAST:  return 'E';
    case MM_SOUTH: return 'S';
    default:       return 'W';
    }
}

static bool in_maze(int x, int y) { return x >= 0 && y >= 0 && x < g_w && y < g_h; }

static bool is_goal(int x, int y)
{
    return x >= g_gx0 && x <= g_gx1 && y >= g_gy0 && y <= g_gy1;
}

/* ==== Maze size and goal ================================================= */

/* Read size and goal from the HAL and check them. Returns false if unusable. */
static bool maze_init(void)
{
    mm_maze_t m = mm_maze();
    int t;

    g_w = m.width;
    g_h = m.height;
    if (g_w < 1 || g_h < 1 || g_w > MM_MAX_SIZE || g_h > MM_MAX_SIZE) {
        mm_log("solver: maze size %dx%d is not supported", g_w, g_h);
        return false;
    }

    g_gx0 = m.goal_x0; g_gy0 = m.goal_y0;
    g_gx1 = m.goal_x1; g_gy1 = m.goal_y1;
    if (g_gx0 > g_gx1) { t = g_gx0; g_gx0 = g_gx1; g_gx1 = t; }
    if (g_gy0 > g_gy1) { t = g_gy0; g_gy0 = g_gy1; g_gy1 = t; }
    if (!in_maze(g_gx0, g_gy0) || !in_maze(g_gx1, g_gy1)) {
        mm_log("solver: goal (%d,%d)-(%d,%d) is outside the maze, using the centre",
               m.goal_x0, m.goal_y0, m.goal_x1, m.goal_y1);
        g_gx0 = (g_w - 1) / 2; g_gx1 = g_w / 2;
        g_gy0 = (g_h - 1) / 2; g_gy1 = g_h / 2;
    }
    return true;
}

/* ==== Drawing in the simulator (does nothing on the robot) =============== */

#if SHOW_DISTANCES
static uint16_t g_label[MM_MAX_SIZE][MM_MAX_SIZE];   /* what each cell shows now */
#define LABEL_NONE 0xFFFEu
#endif

/* Write an unsigned number as text (we avoid stdio.h on purpose). */
static void u16_to_text(unsigned v, char *buf)
{
    char tmp[6];
    int n = 0, i;
    do { tmp[n++] = (char)('0' + v % 10u); v /= 10u; } while (v && n < 5);
    for (i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
}

static void forget_labels(void)
{
#if SHOW_DISTANCES
    int x, y;
    for (x = 0; x < g_w; x++)
        for (y = 0; y < g_h; y++)
            g_label[x][y] = LABEL_NONE;
#endif
}

/* Clear the drawing and colour the goal block. */
static void draw_reset(void)
{
    int x, y;
    mm_clear_marks();
    forget_labels();
    for (x = g_gx0; x <= g_gx1; x++)
        for (y = g_gy0; y <= g_gy1; y++)
            mm_mark(x, y, MM_COLOR_GREEN);
}

/* Show g_dist on the maze; only cells whose number changed are re-sent. */
static void show_distances(void)
{
#if SHOW_DISTANCES
    int x, y;
    char buf[8];
    for (x = 0; x < g_w; x++) {
        for (y = 0; y < g_h; y++) {
            uint16_t v = g_dist[x][y];
            if (g_label[x][y] == v) continue;
            g_label[x][y] = v;
            if (v == DIST_INF) {
                mm_label(x, y, NULL);
            } else {
                u16_to_text(v, buf);
                mm_label(x, y, buf);
            }
        }
    }
#endif
}

/* ==== Map ================================================================ */

/* Empty map: only the outer wall is set (believed, not yet observed). */
static void map_init(void)
{
    int x, y;
    memset(g_walls, 0, sizeof g_walls);
    memset(g_known, 0, sizeof g_known);
    memset(g_sure, 0, sizeof g_sure);
    memset(g_visited, 0, sizeof g_visited);
    g_cautious = false;
    for (x = 0; x < g_w; x++) {
        for (y = 0; y < g_h; y++) {
            if (y == g_h - 1) g_walls[x][y] |= MM_WALL(MM_NORTH);
            if (x == g_w - 1) g_walls[x][y] |= MM_WALL(MM_EAST);
            if (y == 0)       g_walls[x][y] |= MM_WALL(MM_SOUTH);
            if (x == 0)       g_walls[x][y] |= MM_WALL(MM_WEST);
        }
    }
}

/* Record one observation: side d of cell (x,y) is a wall (true) or open (false).
 * The same wall is also the opposite side of the neighbour, so both are set.
 * sure = the observation comes from motion (we drove through the side, or a
 * move was blocked by it). A sensor reading never overrides a sure side; on
 * the simulator the two always agree, on the robot this stops one bad ToF
 * reading from undoing what the wheels already proved. */
static void map_set_side(int x, int y, int d, bool wall, bool sure)
{
    int nx = x + dir_dx(d), ny = y + dir_dy(d);
    uint8_t bit  = MM_WALL(d);
    uint8_t nbit = MM_WALL(dir_back(d));
    bool was_known = (g_known[x][y] & bit) != 0;
    bool was_wall  = (g_walls[x][y] & bit) != 0;
    bool was_sure  = (g_sure[x][y] & bit) != 0;

    if (!in_maze(nx, ny) && !wall) {
        /* The outer wall is always closed: "open" here is a sensor error. */
        mm_log("map: open reading on the outer wall at (%d,%d) %c ignored",
               x, y, dir_name(d));
        wall = true;
    }
    if (was_known && was_wall != wall) {
        if (was_sure && !sure) {
            mm_log("map: sensor says (%d,%d) %c is %s, but motion proved otherwise",
                   x, y, dir_name(d), wall ? "wall" : "open");
            return;
        }
        if (!sure && was_wall) {
            /* Two sensor readings disagree (e.g. the two cells that share this
             * wall). Keeping the wall is the safe choice: if it is wrong and it
             * blocks the way, reopen_cut_walls() will test it by driving. */
            mm_log("map: readings disagree at (%d,%d) %c, keeping the wall",
                   x, y, dir_name(d));
            return;
        }
        mm_log("map: (%d,%d) side %c changed to %s", x, y, dir_name(d),
               wall ? "wall" : "open");
    }

    g_known[x][y] |= bit;
    if (sure) g_sure[x][y] |= bit;
    if (wall) g_walls[x][y] |= bit;
    else      g_walls[x][y] &= (uint8_t)~bit;

    if (in_maze(nx, ny)) {
        g_known[nx][ny] |= nbit;
        if (sure) g_sure[nx][ny] |= nbit;
        if (wall) g_walls[nx][ny] |= nbit;
        else      g_walls[nx][ny] &= (uint8_t)~nbit;
    }

    if (wall && !(was_known && was_wall))
        mm_show_wall(x, y, d);
}

/* Sense the current cell and turn the relative reading into absolute walls. */
static void sense_cell(void)
{
    uint8_t s = mm_sense();
    map_set_side(g_x, g_y, dir_left(g_head),  (s & MM_SEE_LEFT)  != 0, false);
    map_set_side(g_x, g_y, g_head,            (s & MM_SEE_FRONT) != 0, false);
    map_set_side(g_x, g_y, dir_right(g_head), (s & MM_SEE_RIGHT) != 0, false);
    g_visited[g_x][g_y] = true;
}

/* In cautious mode, may this side be part of a multi-cell move? */
static bool side_mergeable(int x, int y, int d)
{
    return !g_cautious || (g_sure[x][y] & MM_WALL(d)) != 0;
}

/* Can the mouse go from (x,y) towards d?
 * PLAN_OPTIMISTIC: unless a wall is believed there (unseen sides count as open).
 * PLAN_KNOWN:      only if the side was observed to be open. */
static bool can_move(int x, int y, int d, plan_t plan)
{
    uint8_t bit = MM_WALL(d);
    if (!in_maze(x + dir_dx(d), y + dir_dy(d))) return false;
    if (g_walls[x][y] & bit) return false;
    if (plan == PLAN_KNOWN && !(g_known[x][y] & bit)) return false;
    return true;
}

/* Report the whole map (required after every run). */
static void report_map(void)
{
    int x, y, seen = 0;
    for (x = 0; x < g_w; x++)
        for (y = 0; y < g_h; y++)
            if (g_visited[x][y]) seen++;
    mm_log("solver: map dump, %d of %d cells visited, t=%lu ms", seen, g_w * g_h,
           (unsigned long)mm_millis());

    mm_map_begin();
    for (y = 0; y < g_h; y++) {
        for (x = 0; x < g_w; x++) {
            uint8_t known = g_known[x][y];
#if REPORT_PERIMETER_AS_KNOWN
            if (y == g_h - 1) known |= MM_WALL(MM_NORTH);
            if (x == g_w - 1) known |= MM_WALL(MM_EAST);
            if (y == 0)       known |= MM_WALL(MM_SOUTH);
            if (x == 0)       known |= MM_WALL(MM_WEST);
#endif
            mm_map_cell(x, y, g_walls[x][y], known);
        }
    }
    mm_map_end();
}

/* ==== Flood fill ========================================================= */

/* Breadth-first flood fill from every cell marked in seed[][].
 * Afterwards dist[x][y] is the number of cells from (x,y) to the nearest seed,
 * or DIST_INF if no seed can be reached. A FIFO queue makes every cell get
 * its smallest distance the first time it is reached. */
static void flood(uint16_t dist[][MM_MAX_SIZE], bool seed[][MM_MAX_SIZE], plan_t plan)
{
    static uint8_t qx[N_CELLS], qy[N_CELLS];
    int head = 0, tail = 0, x, y, d;

    for (x = 0; x < g_w; x++) {
        for (y = 0; y < g_h; y++) {
            if (seed[x][y]) {
                dist[x][y] = 0;
                qx[tail] = (uint8_t)x; qy[tail] = (uint8_t)y; tail++;
            } else {
                dist[x][y] = DIST_INF;
            }
        }
    }

    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        for (d = 0; d < 4; d++) {
            int nx = cx + dir_dx(d), ny = cy + dir_dy(d);
            if (!can_move(cx, cy, d, plan) || dist[nx][ny] != DIST_INF) continue;
            dist[nx][ny] = (uint16_t)(dist[cx][cy] + 1);
            qx[tail] = (uint8_t)nx; qy[tail] = (uint8_t)ny; tail++;
        }
    }
}

/* The neighbour of (x,y) closest to the target, or -1 if none is closer than
 * (x,y) itself. Ties go straight ahead first (no turn), then right, left, back. */
static int best_dir(int x, int y, int heading, uint16_t dist[][MM_MAX_SIZE], plan_t plan)
{
    int order[4];
    int i, best = -1;
    uint16_t best_d = dist[x][y];

    order[0] = heading;
    order[1] = dir_right(heading);
    order[2] = dir_left(heading);
    order[3] = dir_back(heading);
    for (i = 0; i < 4; i++) {
        int d = order[i];
        if (!can_move(x, y, d, plan)) continue;
        if (dist[x + dir_dx(d)][y + dir_dy(d)] < best_d) {
            best = d;
            best_d = dist[x + dir_dx(d)][y + dir_dy(d)];
        }
    }
    return best;
}

/* ==== Targets ============================================================ */

static uint16_t g_to_goal[MM_MAX_SIZE][MM_MAX_SIZE];
static uint16_t g_from_start[MM_MAX_SIZE][MM_MAX_SIZE];
static uint16_t g_known_to_goal[MM_MAX_SIZE][MM_MAX_SIZE];
static bool     g_seed[MM_MAX_SIZE][MM_MAX_SIZE];

static void seed_goal(void)
{
    int x, y;
    memset(g_seed, 0, sizeof g_seed);
    for (x = g_gx0; x <= g_gx1; x++)
        for (y = g_gy0; y <= g_gy1; y++)
            g_seed[x][y] = true;
}

static void seed_start(void)
{
    memset(g_seed, 0, sizeof g_seed);
    g_seed[0][0] = true;
}

/* Shortest-path proof.
 *   L_opt   = shortest start->goal path if every unseen side were open,
 *   L_known = shortest start->goal path through sides seen to be open.
 * No real path is shorter than L_opt, and the known path is real, so when
 * L_known == L_opt the known path is the true shortest path: we are done.
 * Otherwise every optimistic shortest path still crosses an unseen side.
 * A side (c -> n) lies on such a path when
 *   from_start[c] + 1 + to_goal[n] == L_opt.
 * The unvisited cells at both ends of those sides become the targets.
 * Visiting a cell observes all its sides, so every visit removes a target and
 * the phase always ends. Returns the number of targets (0 = proven). */
static int build_proof_targets(void)
{
    int x, y, d, count = 0;
    uint16_t l_opt, l_known;

    seed_goal();
    flood(g_to_goal, g_seed, PLAN_OPTIMISTIC);
    flood(g_known_to_goal, g_seed, PLAN_KNOWN);
    seed_start();
    flood(g_from_start, g_seed, PLAN_OPTIMISTIC);

    l_opt   = g_to_goal[0][0];
    l_known = g_known_to_goal[0][0];
    if (l_opt == DIST_INF || l_known == l_opt) return 0;

    for (x = 0; x < g_w; x++) {
        for (y = 0; y < g_h; y++) {
            for (d = 0; d < 4; d++) {
                int nx = x + dir_dx(d), ny = y + dir_dy(d);
                if (!can_move(x, y, d, PLAN_OPTIMISTIC)) continue;
                if (g_known[x][y] & MM_WALL(d)) continue;          /* already seen */
                if ((long)g_from_start[x][y] + 1 + (long)g_to_goal[nx][ny] != (long)l_opt)
                    continue;
                if (!g_visited[x][y]   && !g_target[x][y])   { g_target[x][y] = true;   count++; }
                if (!g_visited[nx][ny] && !g_target[nx][ny]) { g_target[nx][ny] = true; count++; }
            }
        }
    }
    if (count == 0)
        mm_log("proof: known path %u > optimistic %u but nothing left to visit",
               (unsigned)l_known, (unsigned)l_opt);
    return count;
}

/* Fill g_target[][] for this phase. Returns the number of target cells. */
static int build_targets(target_t t)
{
    int x, y, count = 0;
    memset(g_target, 0, sizeof g_target);
    switch (t) {
    case TARGET_GOAL:
        for (x = g_gx0; x <= g_gx1; x++)
            for (y = g_gy0; y <= g_gy1; y++) { g_target[x][y] = true; count++; }
        break;
    case TARGET_START:
        g_target[0][0] = true;
        count = 1;
        break;
    case TARGET_PROOF:
        count = build_proof_targets();
        break;
    }
    return count;
}

/* ==== Motion (with reset checks) ========================================= */

/* If the operator reset the mouse: pose back to the start, map kept. */
static bool poll_reset(void)
{
    if (!mm_reset_requested()) return false;
    g_x = 0;
    g_y = 0;
    g_head = MM_NORTH;
    g_resets++;
    mm_log("reset #%d: back at (0,0) facing N, map kept", g_resets);
    mm_reset_done();
    return true;
}

/* Lost (position unknown): report the map and wait for the operator. */
static void wait_for_reset(void)
{
    report_map();
    mm_log("solver: position unknown, put the mouse in the start cell and reset");
    while (!poll_reset()) {
        uint32_t t0 = mm_millis();
        while (mm_millis() - t0 < 50u) { /* wait 50 ms between checks */ }
    }
}

/* Turn in place to face absolute direction d. */
static go_t turn_to(int d)
{
    int q = ((d - g_head) % 4 + 4) % 4;       /* quarter turns to the right */
    mm_status_t st;

    if (q == 0) return GO_OK;
    if (poll_reset()) return GO_RESET;
    st = mm_turn(q == 3 ? -1 : q);            /* 1 = right, -1 = left, 2 = around */
    if (st != MM_OK) {
        mm_log("solver: mm_turn failed (status %d) at (%d,%d)", (int)st, g_x, g_y);
        return GO_FAULT;
    }
    g_head = d;
    return GO_OK;
}

/* Drive 'cells' cells straight ahead. */
static go_t forward(int cells)
{
    mm_status_t st;

    if (poll_reset()) return GO_RESET;
    st = mm_forward(cells);
    if (st == MM_OK) {
        int i;
        for (i = 0; i < cells; i++) {          /* every side we crossed is open */
            map_set_side(g_x, g_y, g_head, false, true);
            g_x += dir_dx(g_head);
            g_y += dir_dy(g_head);
        }
        return GO_OK;
    }
    if (st == MM_BLOCKED) {
        if (cells == 1) {
            /* The mouse did not move: the wall ahead was missed by the sensors. */
            mm_log("solver: blocked at (%d,%d) facing %c, adding the wall",
                   g_x, g_y, dir_name(g_head));
            map_set_side(g_x, g_y, g_head, true, true);
            return GO_BLOCKED;
        }
        /* On a multi-cell move we cannot tell where the mouse stopped. From now
         * on, sides not proven by motion are crossed one cell at a time, so the
         * next bump shows exactly which wall the sensors missed. */
        mm_log("solver: blocked during a %d-cell move, switching to cautious moves", cells);
        g_cautious = true;
        wait_for_reset();
        return GO_RESET;
    }
    mm_log("solver: mm_forward(%d) fault at (%d,%d)", cells, g_x, g_y);
    return GO_FAULT;
}

/* ==== Recovering from a wrong "wall" reading ============================= */

/* A real maze always has a route, so when the flood from the target cannot
 * reach the mouse (g_dist is DIST_INF where the mouse is), a wall reading was
 * wrong. The walls that cut the mouse off are the walls between a cell the
 * flood did not reach and a cell it did. Each of them that only the sensors
 * reported (not the outer wall, not proven by motion) is set back to unknown.
 * Flood fill treats unknown as open, so the mouse drives up to it and crosses
 * it one cell at a time: if it is really open the move succeeds, if it is
 * really a wall the move is blocked. Either way the side is then proven by
 * motion and a sensor can no longer undo it. Returns the walls re-opened. */
static int reopen_cut_walls(void)
{
    int x, y, d, n = 0;

    for (x = 0; x < g_w; x++) {
        for (y = 0; y < g_h; y++) {
            for (d = 0; d < 4; d++) {
                int nx = x + dir_dx(d), ny = y + dir_dy(d);
                uint8_t bit = MM_WALL(d), nbit = MM_WALL(dir_back(d));

                if (!in_maze(nx, ny)) continue;                      /* outer wall */
                if (g_dist[x][y] != DIST_INF || g_dist[nx][ny] == DIST_INF)
                    continue;                                        /* not on the cut */
                if (!(g_walls[x][y] & bit) || (g_sure[x][y] & bit))
                    continue;                                        /* open, or proven */

                g_walls[x][y]   &= (uint8_t)~bit;
                g_known[x][y]   &= (uint8_t)~bit;
                g_walls[nx][ny] &= (uint8_t)~nbit;
                g_known[nx][ny] &= (uint8_t)~nbit;
                g_visited[nx][ny] = false;           /* look again when we get there */
                mm_log("solver: wall (%d,%d) %c may be a wrong reading, will try it",
                       x, y, dir_name(d));
                n++;
            }
        }
    }
    return n;
}

/* ==== Exploring with flood fill ========================================== */

/* Drive towards the targets of phase t, sensing every new cell, until the
 * mouse stands on a target (or, for TARGET_PROOF, no target is left).
 * Straight runs through cells we have already visited are driven as one
 * mm_forward(n): those cells are fully known, so there is nothing to sense. */
static run_t explore(target_t t)
{
    for (;;) {
        int d, run, cx, cy;
        go_t g;

        if (poll_reset()) return RUN_RESET;
        if (!g_visited[g_x][g_y]) sense_cell();

        if (build_targets(t) == 0) return RUN_DONE;
        if (g_target[g_x][g_y]) return RUN_DONE;

        flood(g_dist, g_target, PLAN_OPTIMISTIC);
        show_distances();

        d = best_dir(g_x, g_y, g_head, g_dist, PLAN_OPTIMISTIC);
        if (d < 0) {
            if (g_reopens < MAX_REOPENS && reopen_cut_walls() > 0) {
                g_reopens++;
                continue;                    /* plan again with those walls unknown */
            }
            mm_log("solver: no route from (%d,%d) to the target", g_x, g_y);
            return RUN_NO_PATH;
        }

        /* Several cells in one move only through sides already seen open:
         * an unseen side is always crossed on its own, so a wall there blocks
         * a one-cell move and the solver knows exactly where it is. */
        run = 1;
        cx = g_x + dir_dx(d);
        cy = g_y + dir_dy(d);
        if (side_mergeable(g_x, g_y, d) && (g_known[g_x][g_y] & MM_WALL(d))) {
            while (g_visited[cx][cy] && !g_target[cx][cy] &&
                   (g_known[cx][cy] & MM_WALL(d)) &&
                   best_dir(cx, cy, d, g_dist, PLAN_OPTIMISTIC) == d &&
                   side_mergeable(cx, cy, d)) {
                run++;
                cx += dir_dx(d);
                cy += dir_dy(d);
            }
        }

        g = turn_to(d);
        if (g == GO_OK) g = forward(run);
        if (g == GO_RESET) return RUN_RESET;
        if (g == GO_FAULT) return RUN_FAULT;
        /* GO_OK or GO_BLOCKED (a wall was added): sense and plan again */
    }
}

/* ==== Fast runs on known walls =========================================== */
/* A state is (cell, heading). Dijkstra over states finds the cheapest plan
 * from the current pose to any target cell using only sides seen to be open:
 * moving one cell costs SPEED_CELL_COST, a quarter turn SPEED_TURN_COST. */

#define N_STATES (N_CELLS * 4)
#define COST_INF 0xFFFFFFFFu

static uint32_t s_cost[N_STATES];
static int16_t  s_prev[N_STATES];
static bool     s_done[N_STATES];
static int16_t  s_path[N_STATES];
static int8_t   s_seg_dir[N_CELLS];
static uint8_t  s_seg_len[N_CELLS];
static bool     s_seg_open[N_CELLS];   /* may more cells be added to this segment? */

static int st_make(int x, int y, int h) { return (y * MM_MAX_SIZE + x) * 4 + h; }
static int st_x(int s) { return (s / 4) % MM_MAX_SIZE; }
static int st_y(int s) { return (s / 4) / MM_MAX_SIZE; }
static int st_h(int s) { return s % 4; }

static void relax(int from, int to, uint32_t step)
{
    if (s_done[to]) return;
    if (s_cost[from] + step < s_cost[to]) {
        s_cost[to] = s_cost[from] + step;
        s_prev[to] = (int16_t)from;
    }
}

/* Plan into s_path[]. Returns the number of states (0 = no known path). */
static int plan_known(void)
{
    int x, y, h, s, len = 0, end = -1;

    for (x = 0; x < g_w; x++)
        for (y = 0; y < g_h; y++)
            for (h = 0; h < 4; h++) {
                s = st_make(x, y, h);
                s_cost[s] = COST_INF;
                s_prev[s] = -1;
                s_done[s] = false;
            }
    s_cost[st_make(g_x, g_y, g_head)] = 0;

    for (;;) {
        int best = -1;
        for (x = 0; x < g_w; x++)            /* pick the cheapest open state */
            for (y = 0; y < g_h; y++)
                for (h = 0; h < 4; h++) {
                    s = st_make(x, y, h);
                    if (!s_done[s] && s_cost[s] != COST_INF &&
                        (best < 0 || s_cost[s] < s_cost[best]))
                        best = s;
                }
        if (best < 0) return 0;              /* nothing reachable is a target */
        s_done[best] = true;

        x = st_x(best); y = st_y(best); h = st_h(best);
        if (g_target[x][y]) { end = best; break; }

        if (can_move(x, y, h, PLAN_KNOWN))
            relax(best, st_make(x + dir_dx(h), y + dir_dy(h), h), SPEED_CELL_COST);
        relax(best, st_make(x, y, dir_right(h)), SPEED_TURN_COST);
        relax(best, st_make(x, y, dir_left(h)),  SPEED_TURN_COST);
        relax(best, st_make(x, y, dir_back(h)),  2u * SPEED_TURN_COST);
    }

    for (s = end; s >= 0; s = s_prev[s]) len++;          /* walk back ...      */
    h = len;
    for (s = end; s >= 0; s = s_prev[s]) s_path[--h] = (int16_t)s;   /* ... store forwards */
    return len;
}

/* Drive to the targets of phase t on known walls only, merging straight cells
 * into one mm_forward(n). Re-plans if a move is blocked. */
static run_t drive_known(target_t t, bool show_path)
{
    for (;;) {
        int len, i, nseg = 0, cells = 0;
        go_t g = GO_OK;

        if (poll_reset()) return RUN_RESET;
        build_targets(t);
        if (g_target[g_x][g_y]) return RUN_DONE;

        len = plan_known();
        if (len == 0) return RUN_NO_PATH;

        /* states -> straight segments (direction, number of cells).
         * In cautious mode a side not proven by motion gets a segment of its own. */
        for (i = 1; i < len; i++) {
            int a = s_path[i - 1], b = s_path[i], h = st_h(b);
            bool merge_ok;
            if (st_x(a) == st_x(b) && st_y(a) == st_y(b)) continue;   /* a turn */
            cells++;
            merge_ok = side_mergeable(st_x(a), st_y(a), h);
            if (nseg > 0 && s_seg_dir[nseg - 1] == h && merge_ok && s_seg_open[nseg - 1]) {
                s_seg_len[nseg - 1]++;
            } else {
                s_seg_dir[nseg] = (int8_t)h;
                s_seg_len[nseg] = 1;
                s_seg_open[nseg] = merge_ok;
                nseg++;
            }
        }

        if (show_path) {
            draw_reset();
            for (i = 0; i < len; i++)
                if (!is_goal(st_x(s_path[i]), st_y(s_path[i])))
                    mm_mark(st_x(s_path[i]), st_y(s_path[i]), MM_COLOR_CYAN);
            mm_log("speed run: %d cells in %d straight segments, t=%lu ms",
                   cells, nseg, (unsigned long)mm_millis());
        }

        for (i = 0; i < nseg && g == GO_OK; i++) {
            g = turn_to(s_seg_dir[i]);
            if (g == GO_OK) g = forward(s_seg_len[i]);
        }
        if (g == GO_RESET) return RUN_RESET;
        if (g == GO_FAULT) return RUN_FAULT;
        /* GO_OK: loop once more to confirm we are on a target.
         * GO_BLOCKED: the map changed, so plan again. */
    }
}

/* Known-walls run, falling back to exploring if no known path exists. */
static run_t go_fast(target_t t, bool show_path)
{
    run_t r = drive_known(t, show_path);
    if (r == RUN_NO_PATH) {
        mm_log("solver: no known path, exploring instead");
        r = explore(t);
    }
    return r;
}

/* ==== Entry point ======================================================== */

typedef enum {
    PH_TO_GOAL, PH_TO_START, PH_PROVE, PH_RETURN,
    PH_SPEED, PH_SPEED_BACK, PH_FINISH, PH_ABORT
} phase_t;

void solver_run(void)
{
    phase_t ph = PH_TO_GOAL;
    int speed_runs = 0;

    mm_begin(TEAM_ID);
    if (!maze_init()) {
        mm_log("solver: stopped");
        return;
    }
    mm_log("solver: maze %dx%d, goal (%d,%d)-(%d,%d)",
           g_w, g_h, g_gx0, g_gy0, g_gx1, g_gy1);

    map_init();
    g_x = 0; g_y = 0; g_head = MM_NORTH;
    g_resets = 0;
    g_reopens = 0;
    draw_reset();

    while (ph != PH_FINISH && ph != PH_ABORT) {
        run_t r;
        switch (ph) {
        case PH_TO_GOAL:                         /* a) explore to the goal */
            r = explore(TARGET_GOAL);
            if (r == RUN_DONE) {
                mm_log("solver: goal reached at (%d,%d), t=%lu ms",
                       g_x, g_y, (unsigned long)mm_millis());
                report_map();
                ph = PH_TO_START;
            } else if (r != RUN_RESET) {
                ph = PH_ABORT;                   /* a reset just restarts this phase */
            }
            break;

        case PH_TO_START:                        /* b) explore back to the start */
            r = explore(TARGET_START);
            if (r == RUN_DONE || r == RUN_RESET) {
                mm_log("solver: back at the start, t=%lu ms", (unsigned long)mm_millis());
                ph = PROVE_SHORTEST_PATH ? PH_PROVE : PH_SPEED;
            } else {
                ph = PH_ABORT;
            }
            break;

        case PH_PROVE:                           /* c) prove the shortest path */
            r = explore(TARGET_PROOF);
            if (r == RUN_DONE || r == RUN_NO_PATH) {
                mm_log("solver: shortest path proven, t=%lu ms", (unsigned long)mm_millis());
                ph = PH_RETURN;
            } else if (r != RUN_RESET) {
                ph = PH_ABORT;
            }
            break;

        case PH_RETURN:                          /* d) back to the start, fast */
            r = go_fast(TARGET_START, false);
            ph = (r == RUN_DONE || r == RUN_RESET) ? PH_SPEED : PH_ABORT;
            break;

        case PH_SPEED:                           /* e) speed run */
            r = go_fast(TARGET_GOAL, true);
            if (r == RUN_DONE) {
                speed_runs++;
                mm_log("solver: speed run %d finished, t=%lu ms",
                       speed_runs, (unsigned long)mm_millis());
                report_map();
                ph = PH_SPEED_BACK;
            } else if (r != RUN_RESET) {
                ph = PH_ABORT;
            }
            break;

        case PH_SPEED_BACK:                      /* f) back to the start */
            r = go_fast(TARGET_START, false);
            if (r == RUN_DONE || r == RUN_RESET)
                ph = (speed_runs < SPEED_RUNS) ? PH_SPEED : PH_FINISH;
            else
                ph = PH_ABORT;
            break;

        default:
            ph = PH_ABORT;
            break;
        }
    }

    report_map();
    mm_log("solver: %s, %d reset(s), %d wall re-check(s), t=%lu ms",
           ph == PH_FINISH ? "finished" : "stopped",
           g_resets, g_reopens, (unsigned long)mm_millis());
}
