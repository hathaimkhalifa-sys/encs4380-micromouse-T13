#!/usr/bin/env python3
"""
wallfollow_check.py - does a wall-follower solve this maze?

    python wallfollow_check.py tests\\T13_no_wallfollow_16x16.txt

Simulates a left-hand and a right-hand wall-follower (the same rule as the
kit's example_wallfollow.c) from (0,0) facing north. The mouse's next move
depends only on its cell and heading, so as soon as a (cell, heading) state
comes back, the follower is in a loop it can never leave. The goal is read
from MAZE.goal next to the maze, else the centre.
"""
import os
import sys

N, E, S, W = 0, 1, 2, 3
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]


def load_maze(path):
    with open(path) as f:
        lines = [ln.rstrip("\r\n") for ln in f if ln.strip()]
    h = (len(lines) - 1) // 2
    w = (len(lines[0].rstrip()) - 1) // 4

    def ch(r, c):
        return lines[r][c] if c < len(lines[r]) else " "

    walls = {}
    for row in range(h):
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


def load_goal(maze, w, h):
    g = os.path.splitext(maze)[0] + ".goal"
    if os.path.exists(g):
        with open(g) as f:
            v = [int(t) for t in f.read().replace(" ", "").strip().split(",")]
        if len(v) == 2:
            v = v + v
        return min(v[0], v[2]), min(v[1], v[3]), max(v[0], v[2]), max(v[1], v[3])
    return (w - 1) // 2, (h - 1) // 2, w // 2, h // 2


def follow(walls, goal, hand):
    x, y, hd = 0, 0, N
    seen = set()
    cells = {(0, 0)}
    steps = 0
    while True:
        if goal[0] <= x <= goal[2] and goal[1] <= y <= goal[3]:
            return True, steps, cells
        if (x, y, hd) in seen:
            return False, steps, cells
        seen.add((x, y, hd))
        first = (hd + 3) % 4 if hand == "left" else (hd + 1) % 4
        last = (hd + 1) % 4 if hand == "left" else (hd + 3) % 4
        if first not in walls[(x, y)]:
            hd = first
        elif hd not in walls[(x, y)]:
            pass
        elif last not in walls[(x, y)]:
            hd = last
        else:
            hd = (hd + 2) % 4
        x += DX[hd]
        y += DY[hd]
        steps += 1
        cells.add((x, y))


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    maze = sys.argv[1]
    w, h, walls = load_maze(maze)
    goal = load_goal(maze, w, h)
    print("%s: %dx%d, goal (%d,%d)-(%d,%d)" % ((os.path.basename(maze), w, h) + goal))
    solved = False
    for hand in ("left", "right"):
        ok, steps, cells = follow(walls, goal, hand)
        if ok:
            solved = True
            print("  %5s-hand follower: REACHES the goal after %d steps" % (hand, steps))
        else:
            print("  %5s-hand follower: never reaches the goal; after %d steps it repeats the same "
                  "loop forever (visits %d of %d cells)" % (hand, steps, len(cells), w * h))
    print("RESULT: a wall-follower %s this maze" % ("CAN solve" if solved else "CANNOT solve"))


if __name__ == "__main__":
    main()
