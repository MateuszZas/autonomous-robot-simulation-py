import pygame
import random
import heapq

pygame.init()

UI_H      = 60
GRID_SIZE = 40
COLS      = 20
ROWS      = 14
WIDTH     = COLS * GRID_SIZE
HEIGHT    = ROWS * GRID_SIZE + UI_H

screen   = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Cleaner PRO")

clock    = pygame.time.Clock()
font     = pygame.font.SysFont(None, 26)
font_big = pygame.font.SysFont(None, 38)

DIRECTIONS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def in_bounds(x, y):
    return 0 <= x < COLS and 0 <= y < ROWS


def generate_world():
    obstacles = set()
    while len(obstacles) < 30:
        x = random.randint(0, COLS - 1)
        y = random.randint(0, ROWS - 1)
        if (x, y) != (0, 0):
            obstacles.add((x, y))
    return frozenset(obstacles)


def fresh_state():
    return {
        "robot_pos":   (0, 0),
        "robot_map":   [[None] * ROWS for _ in range(COLS)],  # None=unknown, 0=free, 1=wall
        "cleaned":     set(),
        "unreachable": set(),
        "path":        [],
        "phase":       "mapping",  # mapping | cleaning | done
    }


def sense(x, y, real_obstacles, robot_map):
    # 3x3 scan so the robot discovers walls without having to step next to them
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            nx, ny = x + dx, y + dy
            if not in_bounds(nx, ny):
                continue
            if (nx, ny) in real_obstacles:
                robot_map[nx][ny] = 1
            elif robot_map[nx][ny] is None:
                robot_map[nx][ny] = 0


def bfs_to_frontier(start, robot_map):
    # Finds shortest path from start to the nearest unknown cell via known-free cells
    queue   = [start]
    visited = {start}
    parent  = {start: None}

    while queue:
        cx, cy = queue.pop(0)
        for dx, dy in DIRECTIONS:
            nx, ny = cx + dx, cy + dy
            if not in_bounds(nx, ny) or (nx, ny) in visited:
                continue
            if robot_map[nx][ny] == 1:
                continue
            if robot_map[nx][ny] is None:
                path, step = [(nx, ny)], (cx, cy)
                while step is not None:
                    path.append(step)
                    step = parent.get(step)
                path.reverse()
                return path[1:]
            visited.add((nx, ny))
            parent[(nx, ny)] = (cx, cy)
            queue.append((nx, ny))
    return []


def astar(start, goal, robot_map):
    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g = {start: 0}

    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur == goal:
            path, node = [], cur
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path

        for dx, dy in DIRECTIONS:
            nb = (cur[0] + dx, cur[1] + dy)
            if not in_bounds(*nb) or robot_map[nb[0]][nb[1]] != 0:
                continue
            tg = g[cur] + 1
            if nb not in g or tg < g[nb]:
                g[nb] = tg
                heapq.heappush(open_set, (tg + h(nb, goal), nb))
                came_from[nb] = cur
    return []


def find_next_target(robot_map, cleaned, unreachable, pos):
    # Try candidates nearest-first; mark each with no A* path as unreachable immediately
    candidates = sorted(
        [(x, y) for x in range(COLS) for y in range(ROWS)
         if robot_map[x][y] == 0
         and (x, y) not in cleaned
         and (x, y) not in unreachable],
        key=lambda t: abs(t[0] - pos[0]) + abs(t[1] - pos[1])
    )
    for target in candidates:
        path = astar(pos, target, robot_map)
        if path:
            return target, path
        unreachable.add(target)
    return None, []


rooms      = [generate_world()]
known_maps = [None]  # stored map per room after first full mapping pass
room_idx   = 0
state      = fresh_state()


def load_room(idx, repeat=False):
    global room_idx, state
    room_idx = idx
    state = fresh_state()
    if repeat and known_maps[idx] is not None:
        state["robot_map"] = [row[:] for row in known_maps[idx]]
        state["phase"]     = "cleaning"


running = True
while running:
    screen.fill((10, 10, 22))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN and state["phase"] == "done":
            if event.key == pygame.K_r:
                load_room(room_idx, repeat=True)
            elif event.key == pygame.K_n:
                rooms.append(generate_world())
                known_maps.append(None)
                load_room(len(rooms) - 1)
            elif event.key == pygame.K_RIGHT:
                if room_idx + 1 >= len(rooms):
                    rooms.append(generate_world())
                    known_maps.append(None)
                load_room(room_idx + 1, repeat=True)
            elif event.key == pygame.K_LEFT and room_idx > 0:
                load_room(room_idx - 1, repeat=True)

    s           = state
    phase       = s["phase"]
    robot_map   = s["robot_map"]
    cleaned     = s["cleaned"]
    unreachable = s["unreachable"]
    path        = s["path"]
    obs         = rooms[room_idx]

    if phase != "done":
        x, y = s["robot_pos"]
        sense(x, y, obs, robot_map)

        if phase == "mapping":
            if not path:
                path = bfs_to_frontier((x, y), robot_map)
                s["path"] = path
            if path:
                s["robot_pos"] = path.pop(0)
            else:
                known_maps[room_idx] = [row[:] for row in robot_map]
                s["phase"] = "cleaning"

        elif phase == "cleaning":
            cleaned.add(s["robot_pos"])
            if not path:
                _, path = find_next_target(robot_map, cleaned, unreachable, s["robot_pos"])
                s["path"] = path
            if path:
                s["robot_pos"] = path.pop(0)
            else:
                s["phase"] = "done"

    for i in range(COLS):
        for j in range(ROWS):
            rect = (i * GRID_SIZE, j * GRID_SIZE, GRID_SIZE - 1, GRID_SIZE - 1)
            val  = robot_map[i][j]
            if val is None:
                color = (18, 18, 30)
            elif val == 1:
                color = (190, 35, 35)
            elif (i, j) in cleaned:
                color = (25, 90, 25)
            else:
                color = (45, 45, 100)
            pygame.draw.rect(screen, color, rect)

    for (px, py) in path:
        pygame.draw.rect(screen, (65, 65, 150),
                         (px * GRID_SIZE + 10, py * GRID_SIZE + 10, GRID_SIZE - 20, GRID_SIZE - 20))

    rx, ry = s["robot_pos"]
    pygame.draw.rect(screen, (0, 230, 90),
                     (rx * GRID_SIZE + 4, ry * GRID_SIZE + 4, GRID_SIZE - 8, GRID_SIZE - 8))

    ui_y   = ROWS * GRID_SIZE
    mapped = known_maps[room_idx] is not None
    total  = sum(1 for i in range(COLS) for j in range(ROWS) if robot_map[i][j] == 0)

    pygame.draw.rect(screen, (20, 20, 38), (0, ui_y, WIDTH, UI_H))
    screen.blit(font.render(f"Room {room_idx + 1}/{len(rooms)}", True, (160, 160, 215)), (10, ui_y + 6))
    screen.blit(font.render(f"Phase: {phase}", True, (220, 220, 220)),                   (10, ui_y + 30))
    screen.blit(font.render(f"Cleaned: {len(cleaned)}/{total}", True, (100, 220, 120)),  (195, ui_y + 6))
    screen.blit(font.render(f"Mapped: {'yes' if mapped else 'no'}",
                            True, (100, 200, 100) if mapped else (180, 80, 80)),          (195, ui_y + 30))

    legend = [((18,18,30),"unkn"), ((190,35,35),"wall"),
              ((45,45,100),"free"), ((25,90,25),"clean"), ((0,230,90),"robot")]
    lx = 360
    for color, label in legend:
        pygame.draw.rect(screen, color, (lx, ui_y + 22, 13, 13))
        screen.blit(font.render(label, True, (170, 170, 170)), (lx + 16, ui_y + 20))
        lx += 76

    if phase == "done":
        overlay = pygame.Surface((WIDTH, 52), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 178))
        gy = ROWS * GRID_SIZE // 2 - 26
        screen.blit(overlay, (0, gy))
        msg = font_big.render("Done!   R=repeat   N=new   < > switch room", True, (0, 255, 110))
        screen.blit(msg, msg.get_rect(center=(WIDTH // 2, gy + 26)))

    pygame.display.flip()
    clock.tick(20)

pygame.quit()