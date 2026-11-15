#!/usr/bin/env python3
"""
mm_check.py - offline tester for an ENCS4380 micromouse solver.

Three ways to use it:

1. Check what mms printed (no solver is run):
     python mm_check.py --dump RUN.txt MAZE.txt
   RUN.txt is the Run Output you copied from mms (Ctrl+A, Ctrl+C, paste, save).
   Prints the real maze next to a drawing of your map dump, lists every wrong
   side, checks the CRC, and, for the full solver, checks that the run used the
   maze's goal and that the speed run was a shortest path.

2. Build the evidence table from many saved runs:
     python mm_check.py --table results [--mazes mazes,tests]
   For every results\\NAME.txt it finds the maze NAME.txt in the maze folders,
   checks it as in 1, prints one table, and writes results\\results_table.csv.

3. Run the solver here, playing the part of mms:
     python mm_check.py SOLVER MAZE.txt [--goal x0,y0,x1,y1] [--reset-at 25,80]
                        [--miss X,Y,D] [--phantom X,Y,D] [--once]
                        [--log] [--map-only] [--save-log FILE]
   --reset-at  press Reset after the N-th movement command (comma list).
   --miss      the sensor misses the wall on side D (N/E/S/W) of cell (X,Y):
               it reports "open" there. Repeat the option for more faults.
   --phantom   the sensor sees a wall that is not there on side D of (X,Y).
   --once      each fault happens only the first time (default: every time).
   With faults, crashes are expected, Reset is pressed for the operator after
   a blocked multi-cell move, and the report shows how the solver recovered.

This is NOT the official grader. Always do the final check in mms itself.
Exit code 0 = pass, 1 = fail.
"""
import argparse
import csv
import glob
import os
import re
import subprocess
import sys
from collections import deque

N, E, S, W = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]
DIR_LETTERS = "NESW"


# ---- maze files ------------------------------------------------------------

def load_maze(path):
    """Read an mms 'map' maze (+---+ text). Returns (w, h, walls) where
    walls[(x, y)] is a set of directions that have a wall; (0,0) is bottom-left."""
    with open(path) as f:
        lines = [ln.rstrip("\n").rstrip("\r") for ln in f if ln.strip()]
    h = (len(lines) - 1) // 2
    w = (len(lines[0].rstrip()) - 1) // 4

    def ch(r, c):
        return lines[r][c] if c < len(lines[r]) else " "

    walls = {}
    for row in range(h):            # text row 0 is the north row
        y = h - 1 - row
        for x in range(w):
            s = set()
            if ch(2 * row, 4 * x + 2) != " ":
                s.add(N)
            if ch(2 * row + 2, 4 * x + 2) != " ":
                s.add(S)
            if ch(2 * row + 1, 4 * x) != " ":
                s.add(W)
            if ch(2 * row + 1, 4 * x + 4) != " ":
                s.add(E)
            walls[(x, y)] = s
    return w, h, walls


def parse_goal_text(text):
    v = [int(t) for t in text.replace(" ", "").split(",")]
    if len(v) == 2:
        v = v + v
    x0, y0, x1, y1 = v
    return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)


def load_goal(maze_path, w, h, goal_arg=None):
    """Goal for a maze: --goal, else MAZE.goal next to the maze, else the centre
    (the same rule as hal_sim.c). Returns (x0, y0, x1, y1, text or None)."""
    text = goal_arg
    if text is None:
        gpath = os.path.splitext(maze_path)[0] + ".goal"
        if os.path.exists(gpath):
            with open(gpath) as f:
                text = f.read().strip()
    if text:
        return parse_goal_text(text) + (text,)
    return (w - 1) // 2, (h - 1) // 2, w // 2, h // 2, None


def shortest(w, h, walls, goal):
    x0, y0, x1, y1 = goal
    dist = {(0, 0): 0}
    q = deque([(0, 0)])
    while q:
        c = q.popleft()
        if x0 <= c[0] <= x1 and y0 <= c[1] <= y1:
            return dist[c]
        for d in range(4):
            if d in walls[c]:
                continue
            n = (c[0] + DX[d], c[1] + DY[d])
            if n not in dist:
                dist[n] = dist[c] + 1
                q.append(n)
    return None


# ---- map dump --------------------------------------------------------------

def crc16(crc, s):
    for b in s.encode():
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def clean_lines(lines):
    """Text copied from mms may carry extra text before the '@'."""
    return [ln[ln.find("@"):].rstrip() if "@" in ln else ln.rstrip() for ln in lines]


def check_dump(lines, w, h, walls):
    """Check the LAST @MAP block. Returns (ok, message, stats)."""
    lines = clean_lines(lines)
    start = None
    for i, ln in enumerate(lines):
        if ln.startswith("@MAP "):
            start = i
    if start is None:
        return False, "no @MAP block", {}
    crc = 0xFFFF
    cells = {}
    end = None
    for ln in lines[start + 1:]:
        if ln.startswith("@C "):
            crc = crc16(crc, ln)
            p = ln.split()
            cells[(int(p[1]), int(p[2]))] = p[3:7]
        elif ln.startswith("@END"):
            end = ln
            break
    if end is None:
        return False, "no @END line (dump cut off?)", {}
    stats = {"cells": cells, "crc_ok": True}
    want = "@END n=%d crc=%04X" % (len(cells), crc)
    crc_msg = None
    if end.strip() != want:
        stats["crc_ok"] = False
        crc_msg = "bad @END (dump edited or cut?): got '%s', expected '%s'" % (end.strip(), want)
    missing = [(x, y) for x in range(w) for y in range(h) if (x, y) not in cells]
    if missing:
        return False, "%d cells missing from the dump, e.g. %s" % (len(missing), missing[0]), stats
    wrong, known, unknown = [], 0, 0
    for (x, y), sides in sorted(cells.items()):
        for d in range(4):
            v = sides[d]
            if v == "?":
                unknown += 1
                continue
            known += 1
            truth = "1" if d in walls[(x, y)] else "0"
            if v != truth:
                wrong.append((x, y, DIR_LETTERS[d], v, truth))
    stats.update({"known_sides": known, "unknown_sides": unknown,
                  "wrong_sides": len(wrong), "wrong": wrong})
    if wrong:
        msg = "%d wrong sides, e.g. cell %s" % (len(wrong), wrong[0])
        return False, msg + ("; " + crc_msg if crc_msg else ""), stats
    if crc_msg:
        return False, crc_msg, stats
    return True, "map dump OK", stats


def draw_dump(cells, w, h):
    """Draw a dump in the same +---+ style as the maze files.
    wall = '---' or '|', open = blank, unknown = ' ? ' or '?'."""
    def side(x, y, d):
        if (x, y) not in cells:
            return "?"
        return cells[(x, y)][d]

    def join(a, b):            # the two cells that share a wall
        known = [v for v in (a, b) if v != "?"]
        if not known:
            return "?"
        return "1" if "1" in known else "0"

    out = []
    for row in range(h):
        y = h - 1 - row
        top = "+"
        for x in range(w):
            v = join(side(x, y, N), side(x, y + 1, S) if y + 1 < h else "?")
            top += {"1": "---", "0": "   ", "?": " ? "}[v] + "+"
        mid = ""
        for x in range(w):
            v = join(side(x, y, W), side(x - 1, y, E) if x > 0 else "?")
            mid += {"1": "|", "0": " ", "?": "?"}[v] + "   "
        v = side(w - 1, y, E)
        mid += {"1": "|", "0": " ", "?": "?"}[v]
        out += [top, mid]
    bottom = "+"
    for x in range(w):
        bottom += {"1": "---", "0": "   ", "?": " ? "}[side(x, 0, S)] + "+"
    out.append(bottom)
    return out


# ---- checking a saved mms Run Output -------------------------------------

HAL_RE = re.compile(r"mm: HAL .*?team (\S+), \S+, maze (\d+)x(\d+), goal \((\d+),(\d+)\)-\((\d+),(\d+)\)")
SPEED_RE = re.compile(r"speed run: (\d+) cells")


def read_text(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return [ln.rstrip("\r\n").lstrip("﻿") for ln in f]


def analyze_output(lines, maze):
    """Check one saved Run Output against its maze. Returns a dict."""
    w, h, walls = load_maze(maze)
    exp = load_goal(maze, w, h)
    r = {"maze": os.path.splitext(os.path.basename(maze))[0], "size": "%dx%d" % (w, h),
         "w": w, "h": h, "walls": walls, "issues": []}

    hdr = None
    for ln in lines:
        if "@MAP " in ln:
            hdr = ln[ln.find("@MAP "):].split()
    info = dict(t.split("=", 1) for t in (hdr or [])[2:] if "=" in t)
    r["team"] = info.get("team", "?")
    if info.get("w") and info.get("h") and (int(info["w"]), int(info["h"])) != (w, h):
        r["issues"].append("dump is from a %sx%s maze, but %s is %dx%d: wrong maze file?"
                           % (info["w"], info["h"], os.path.basename(maze), w, h))
        r["verdict"] = "FAIL"
        return r

    used = None
    for ln in lines:
        m = HAL_RE.search(ln)
        if m:
            used = tuple(int(v) for v in m.groups()[3:7])
    goal = used or exp[:4]
    r["goal"] = "(%d,%d)" % goal[:2] if goal[:2] == goal[2:] else "(%d,%d)-(%d,%d)" % goal
    if used and used != exp[:4]:
        want = "(%d,%d)-(%d,%d)" % exp[:4]
        if exp[4]:
            r["issues"].append("the run used goal %s but %s.goal says %s: copy the .goal file to "
                               "mm_goal.txt before Run" % (r["goal"], r["maze"], want))
        else:
            r["issues"].append("the run used goal %s but this maze has no .goal file (centre %s): "
                               "delete mm_goal.txt" % (r["goal"], want))
    r["shortest"] = shortest(w, h, walls, goal)

    speeds = [int(m.group(1)) for ln in lines for m in [SPEED_RE.search(ln)] if m]
    r["full_solver"] = bool(speeds) or any("solver: goal reached" in ln for ln in lines)
    r["speed"] = speeds[-1] if speeds else None
    r["finished"] = any("solver: finished" in ln for ln in lines)
    if r["full_solver"]:
        if r["speed"] is None:
            r["issues"].append("no speed run in the output")
        elif r["shortest"] is not None and r["speed"] != r["shortest"]:
            r["issues"].append("speed run %d cells, shortest is %d" % (r["speed"], r["shortest"]))
        if not r["finished"]:
            r["issues"].append("no 'solver: finished' line (run stopped or output cut?)")

    ok, msg, stats = check_dump(lines, w, h, walls)
    r.update(stats)
    r["dump_ok"] = ok
    if not ok:
        r["issues"].append(msg)
    r["verdict"] = "PASS" if not r["issues"] else "FAIL"
    return r


def check_dump_file(dump_path, maze):
    """--dump mode: compare a Run Output copied from mms with the real maze."""
    lines = read_text(dump_path)
    r = analyze_output(lines, maze)
    print("maze %s %s, team %s" % (r["maze"], r["size"], r.get("team", "?")))
    if "cells" in r:
        with open(maze) as f:
            real = [ln.rstrip("\r\n") for ln in f if ln.strip()]
        mine = draw_dump(r["cells"], r["w"], r["h"])
        width = max(len(s) for s in real)
        print("%-*s   %s" % (width, "REAL MAZE (" + os.path.basename(maze) + ")", "YOUR MAP DUMP"))
        for a, b in zip(real, mine):
            print("%-*s   %s" % (width, a, b))
        print()
    if "goal" in r:
        print("goal %s, shortest path %s cells" % (r["goal"], r["shortest"]))
    if r.get("full_solver"):
        print("speed run: %s cells" % r["speed"])
    if "known_sides" in r:
        print("sides: %d known, %d unknown (?), %d wrong"
              % (r["known_sides"], r["unknown_sides"], r["wrong_sides"]))
        for x, y, d, v, t in r["wrong"][:20]:
            print("  WRONG: cell (%d,%d) side %s says %s, the maze has %s"
                  % (x, y, d, "wall" if v == "1" else "open", "wall" if t == "1" else "open"))
    if r["verdict"] == "PASS":
        print("PASS: " + ("goal, speed run and map dump OK" if r.get("full_solver") else "map dump OK"))
        return True
    for q in r["issues"]:
        print("FAIL: " + q)
    return False


def table_mode(results_dir, maze_dirs):
    files = sorted(f for f in glob.glob(os.path.join(results_dir, "*.txt")))
    rows = []
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0]
        maze = None
        for d in maze_dirs:
            cand = os.path.join(d, name + ".txt")
            if os.path.exists(cand):
                maze = cand
                break
        if maze is None:
            rows.append({"maze": name, "verdict": "FAIL",
                         "issues": ["no maze %s.txt in %s" % (name, ", ".join(maze_dirs))]})
            continue
        rows.append(analyze_output(read_text(f), maze))
    if not rows:
        print("no .txt files in %s" % results_dir)
        return False

    cols = ["Maze", "Size", "Goal", "Shortest", "Speed run", "Known", "Unknown", "Wrong", "Team", "Result"]
    table = []
    for r in rows:
        table.append([r["maze"], r.get("size", ""), r.get("goal", ""), str(r.get("shortest", "")),
                      "" if r.get("speed") is None else str(r["speed"]),
                      str(r.get("known_sides", "")), str(r.get("unknown_sides", "")),
                      str(r.get("wrong_sides", "")), r.get("team", ""), r["verdict"]])
    widths = [max(len(c), *(len(row[i]) for row in table)) for i, c in enumerate(cols)]
    line = "  ".join(c.ljust(widths[i]) for i, c in enumerate(cols))
    print(line)
    print("-" * len(line))
    for row in table:
        print("  ".join(v.ljust(widths[i]) for i, v in enumerate(row)))
    print()
    for r in rows:
        for q in r["issues"]:
            print("%s: %s" % (r["maze"], q))
    out = os.path.join(results_dir, "results_table.csv")
    with open(out, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(cols)
        wr.writerows(table)
    npass = sum(r["verdict"] == "PASS" for r in rows)
    print("%d of %d PASS. Table saved to %s" % (npass, len(rows), out))
    return npass == len(rows)


# ---- running the solver here (playing mms) --------------------------------

def parse_fault(text, kind):
    p = text.replace(" ", "").split(",")
    if len(p) != 3 or p[2].upper() not in DIR_LETTERS:
        raise ValueError("--%s wants X,Y,D with D one of N,E,S,W, e.g. 2,3,E" % kind)
    return {"kind": kind, "x": int(p[0]), "y": int(p[1]), "d": DIR_LETTERS.index(p[2].upper()),
            "used": 0}


def run(solver, maze, goal_arg=None, reset_at=(), max_cmds=200000, show_log=False,
        faults=(), once=False):
    w, h, walls = load_maze(maze)
    gx0, gy0, gx1, gy1, goal_text = load_goal(maze, w, h, goal_arg)
    goal = (gx0, gy0, gx1, gy1)
    best = shortest(w, h, walls, goal)
    faults = [dict(f) for f in faults]
    for f in faults:
        if (f["x"], f["y"]) not in walls:
            raise ValueError("fault cell (%d,%d) is outside the %dx%d maze" % (f["x"], f["y"], w, h))
        real = f["d"] in walls[(f["x"], f["y"])]
        if f["kind"] == "miss" and not real:
            raise ValueError("--miss %d,%d,%s: there is no wall there to miss"
                             % (f["x"], f["y"], DIR_LETTERS[f["d"]]))
        if f["kind"] == "phantom" and real:
            raise ValueError("--phantom %d,%d,%s: there is already a wall there"
                             % (f["x"], f["y"], DIR_LETTERS[f["d"]]))

    env = dict(os.environ)
    if goal_text:
        env["MM_GOAL"] = goal_text
    else:
        env.pop("MM_GOAL", None)
    sdir = os.path.dirname(os.path.abspath(solver))
    errpath = os.path.join(sdir, ".mm_check_stderr.txt")
    errf = open(errpath, "w")
    p = subprocess.Popen([os.path.abspath(solver)], cwd=sdir, env=env,
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=errf, text=True, bufsize=1)

    x, y, hd = 0, 0, N
    reset_pending = False
    moves = 0
    resets_done = 0
    reset_at = sorted(reset_at)
    in_run, run_cells, run_turns = True, 0, 0      # a run: leave start -> reach goal
    runs = []
    crashes = 0
    cmds = 0
    total_cells = 0
    lies = 0
    error = None

    def at_goal():
        return gx0 <= x <= gx1 and gy0 <= y <= gy1

    def reply(s):
        p.stdin.write(s + "\n")
        p.stdin.flush()

    def note_move():
        nonlocal moves, reset_pending
        moves += 1
        if reset_at and moves >= reset_at[0]:
            reset_at.pop(0)
            reset_pending = True

    try:
        for line in p.stdout:
            cmds += 1
            if cmds > max_cmds:
                error = "more than %d commands (endless loop?)" % max_cmds
                break
            t = line.split()
            if not t:
                continue
            c = t[0]
            if c == "mazeWidth":
                reply(str(w))
            elif c == "mazeHeight":
                reply(str(h))
            elif c in ("wallFront", "wallRight", "wallLeft", "wallBack"):
                rel = {"wallFront": 0, "wallRight": 1, "wallBack": 2, "wallLeft": 3}[c]
                d = (hd + rel) % 4
                seen = d in walls[(x, y)]
                for f in faults:
                    if (f["x"], f["y"], f["d"]) == (x, y, d) and not (once and f["used"]):
                        seen = (f["kind"] == "phantom")
                        f["used"] += 1
                        lies += 1
                reply("true" if seen else "false")
            elif c == "moveForward":
                n = int(t[1]) if len(t) > 1 else 1
                note_move()
                ok = n >= 1
                moved = 0
                while ok and moved < n:
                    if hd in walls[(x, y)]:
                        ok = False
                        break
                    x += DX[hd]
                    y += DY[hd]
                    moved += 1
                    total_cells += 1
                    if in_run:
                        run_cells += 1
                        if at_goal():
                            runs.append((run_cells, run_turns))
                            in_run = False
                    if (x, y) == (0, 0):          # every pass through the start begins a new run
                        in_run, run_cells, run_turns = True, 0, 0
                if not ok:
                    crashes += 1
                    if faults and n > 1:
                        reset_pending = True      # the operator puts the lost mouse back
                reply("ack" if ok else "crash")
            elif c in ("turnRight", "turnRight90", "turnLeft", "turnLeft90"):
                note_move()
                hd = (hd + (1 if c.startswith("turnRight") else 3)) % 4
                if in_run:
                    run_turns += 1
                reply("ack")
            elif c == "wasReset":
                reply("true" if reset_pending else "false")
            elif c == "ackReset":
                if reset_pending:
                    resets_done += 1
                reset_pending = False
                x, y, hd = 0, 0, N
                in_run, run_cells, run_turns = True, 0, 0
                reply("ack")
            elif c == "getStat":
                reply("-1")
            elif c in ("setWall", "clearWall", "setColor", "clearColor", "clearAllColor",
                       "setText", "clearText", "clearAllText"):
                if c == "setWall" and not faults:
                    wx, wy, wd = int(t[1]), int(t[2]), "nesw".index(t[3])
                    if wd not in walls[(wx, wy)]:
                        error = "setWall %s %s %s but there is no wall there" % (t[1], t[2], t[3])
                if c == "setText" and len(t) > 3 and len(t[3]) > 10:
                    error = "setText longer than 10 characters"
            else:
                error = "unknown command from solver: %r" % line.strip()
                break
    except BrokenPipeError:
        pass
    finally:
        try:
            p.stdin.close()
        except Exception:
            pass
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
            error = error or "solver did not exit"
        errf.close()

    with open(errpath) as f:
        log = [ln.rstrip("\n") for ln in f]
    os.remove(errpath)
    if show_log:
        for ln in log:
            if not ln.startswith("@C "):
                print("   | " + ln)

    ok_dump, dump_msg, stats = check_dump(log, w, h, walls)
    problems = []
    if error:
        problems.append(error)
    if p.returncode not in (0, None):
        problems.append("solver exit code %s" % p.returncode)
    if crashes and not faults:
        problems.append("%d crash(es) into walls" % crashes)
    start_in_goal = gx0 <= 0 <= gx1 and gy0 <= 0 <= gy1
    if best is not None and not runs and not start_in_goal:
        problems.append("never reached the goal")
    if runs and best is not None and not start_in_goal and runs[-1][0] != best:
        problems.append("last run %d cells, shortest is %d" % (runs[-1][0], best))
    if not ok_dump:
        problems.append(dump_msg)

    return {
        "maze": os.path.basename(maze), "size": "%dx%d" % (w, h),
        "goal": "(%d,%d)-(%d,%d)" % goal, "shortest": best,
        "runs": runs, "cells_driven": total_cells, "commands": cmds,
        "resets": resets_done, "crashes": crashes, "lies": lies,
        "finished": any("solver: finished" in ln for ln in log),
        **stats, "problems": problems, "log": log,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", metavar="SOLVER MAZE | MAZE")
    ap.add_argument("--goal")
    ap.add_argument("--reset-at", default="")
    ap.add_argument("--max-cmds", type=int, default=200000)
    ap.add_argument("--log", action="store_true")
    ap.add_argument("--map-only", action="store_true",
                    help="check only the map dump and crashes (Task 1)")
    ap.add_argument("--save-log", metavar="FILE", help="save the solver's log and dump to FILE")
    ap.add_argument("--dump", metavar="FILE",
                    help="check a Run Output copied from mms (FILE) against MAZE; no solver is run")
    ap.add_argument("--table", metavar="RESULTS_DIR",
                    help="check every RESULTS_DIR\\NAME.txt against maze NAME.txt and print a table")
    ap.add_argument("--mazes", default="mazes,tests",
                    help="folders to look for mazes in, for --table (default: mazes,tests)")
    ap.add_argument("--miss", action="append", default=[], metavar="X,Y,D")
    ap.add_argument("--phantom", action="append", default=[], metavar="X,Y,D")
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()

    if a.table:
        sys.exit(0 if table_mode(a.table, [d for d in a.mazes.split(",") if d]) else 1)
    if a.dump:
        if not a.paths:
            ap.error("--dump needs the maze file too: --dump RUN.txt MAZE.txt")
        sys.exit(0 if check_dump_file(a.dump, a.paths[-1]) else 1)
    if len(a.paths) != 2:
        ap.error("give SOLVER and MAZE (or --dump RUN.txt MAZE, or --table RESULTS_DIR)")
    solver, maze = a.paths
    try:
        faults = [parse_fault(v, "miss") for v in a.miss] + [parse_fault(v, "phantom") for v in a.phantom]
        resets = [int(v) for v in a.reset_at.split(",") if v.strip()]
        r = run(solver, maze, a.goal, resets, a.max_cmds, a.log, faults, a.once)
    except ValueError as e:
        print("error: %s" % e)
        sys.exit(2)
    if a.save_log:
        with open(a.save_log, "w") as f:
            f.write("\n".join(r["log"]) + "\n")
    if a.map_only:
        r["problems"] = [q for q in r["problems"]
                         if not q.startswith("last run") and q != "never reached the goal"]
    runs = ", ".join("%d cells/%d turns" % rt for rt in r["runs"]) or "none"
    print("%s %s goal %s  shortest=%s" % (r["maze"], r["size"], r["goal"], r["shortest"]))
    print("  runs to goal: %s" % runs)
    print("  cells driven: %d, resets: %d, map: %s known / %s unknown / %s wrong"
          % (r["cells_driven"], r["resets"], r.get("known_sides"), r.get("unknown_sides"),
             r.get("wrong_sides")))

    if faults:
        print("  faults: %s, wrong readings given: %d, crashes: %d"
              % (", ".join("%s %d,%d,%s" % (f["kind"], f["x"], f["y"], DIR_LETTERS[f["d"]])
                           for f in faults), r["lies"], r["crashes"]))
        keys = ("blocked", "wrong reading", "no route", "changed to", "motion proved", "disagree",
                "cautious", "reset #", "position unknown")
        events = [ln for ln in r["log"] if any(k in ln for k in keys)]
        for ln in events[:15]:
            print("    solver: " + ln.replace("solver: ", ""))
        if len(events) > 15:
            print("    ... %d more" % (len(events) - 15))
        for x, y, d, v, t in r.get("wrong", [])[:10]:
            print("  map still wrong at cell (%d,%d) side %s (says %s, maze has %s)"
                  % (x, y, d, "wall" if v == "1" else "open", "wall" if t == "1" else "open"))
        reached = bool(r["runs"])
        if reached and r["finished"]:
            last = r["runs"][-1][0]
            print("  RECOVERED: goal reached, solver finished; last run %d cells (shortest %s)"
                  % (last, r["shortest"]))
            sys.exit(0)
        print("  NOT RECOVERED: " + "; ".join(r["problems"] or ["solver did not finish"]))
        sys.exit(1)

    if r["problems"]:
        print("  FAIL: " + "; ".join(r["problems"]))
        sys.exit(1)
    print("  PASS")


if __name__ == "__main__":
    main()
