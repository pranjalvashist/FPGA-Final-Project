MAX_DIM = 7
MAX_CELLS = MAX_DIM * MAX_DIM


def find_path(dim, blocked, x0, y0, xt, yt):
    """Fixed-capacity BFS, suitable as a reference for an FPGA FSM.

    blocked[y][x] is 1 for obstacles. Returns (path, steps), or (None, None).
    """
    if not (2 <= dim <= MAX_DIM):
        raise ValueError('Grid dimension must be between 2 and 7')
    if len(blocked) != dim or any(len(row) != dim for row in blocked):
        raise ValueError('blocked must be a dim x dim grid')
    if not all(0 <= v < dim for v in (x0, y0, xt, yt)):
        raise ValueError('Start and target must be inside grid')
    if blocked[y0][x0] or blocked[yt][xt]:
        return None, None

    # All buffers have fixed maximum capacity (49).
    visited = [0] * MAX_CELLS
    parent = [-1] * MAX_CELLS
    queue = [0] * MAX_CELLS
    path_buffer = [0] * MAX_CELLS
    head = 0
    tail = 0

    start = y0 * dim + x0
    target = yt * dim + xt
    visited[start] = 1
    queue[tail] = start
    tail += 1

    # Deterministic neighbor priority: right, up, left, down.
    dx = (1, 0, -1, 0)
    dy = (0, 1, 0, -1)

    while head < tail:
        current = queue[head]
        head += 1
        if current == target:
            break
        x = current % dim
        y = current // dim
        for direction in range(4):
            nx = x + dx[direction]
            ny = y + dy[direction]
            if nx < 0 or nx >= dim or ny < 0 or ny >= dim:
                continue
            if blocked[ny][nx]:
                continue
            neighbor = ny * dim + nx
            if visited[neighbor]:
                continue
            visited[neighbor] = 1
            parent[neighbor] = current
            queue[tail] = neighbor
            tail += 1

    if not visited[target]:
        return None, None

    # Backtrack into fixed buffer; output start -> target.
    length = 0
    current = target
    while current != -1:
        path_buffer[length] = current
        length += 1
        current = parent[current]

    path = []
    for index in range(length - 1, -1, -1):
        cell = path_buffer[index]
        path.append((cell % dim, cell // dim))
    return path, length - 1


def show_grid(dim, blocked, path, start, target):
    route = set(path or [])
    for y in range(dim - 1, -1, -1):
        line = []
        for x in range(dim):
            p = (x, y)
            if p == start:
                line.append('S')
            elif p == target:
                line.append('T')
            elif blocked[y][x]:
                line.append('#')
            elif p in route:
                line.append('*')
            else:
                line.append('.')
        print(' '.join(line))


if __name__ == '__main__':
    dim = 5
    blocked = [[0] * dim for _ in range(dim)]
    blocked[2][1] = 1
    blocked[3][3] = 1
    start = (0, 0)
    target = (4, 4)
    path, steps = find_path(dim, blocked, *start, *target)
    show_grid(dim, blocked, path, start, target)
    print('Path:', path)
    print('Steps:', steps)
