import heapq

def bidirectional_dijkstra(grid, start, end):
    rows, cols = grid.rows, grid.cols

    start_pos = start
    end_pos = end

    # Min-heaps for forward and backward searches
    forward_queue = [(0, start_pos)]
    backward_queue = [(0, end_pos)]

    # Distance maps
    forward_dist = {start_pos: 0}
    backward_dist = {end_pos: 0}

    # Visited sets
    forward_visited = {}
    backward_visited = {}

    # Predecessor maps for path reconstruction
    forward_prev = {}
    backward_prev = {}

    meeting_point = None

    while forward_queue and backward_queue:
        # Forward step
        if forward_queue:
            f_dist, f_node = heapq.heappop(forward_queue)
            if f_node in forward_visited:
                continue
            forward_visited[f_node] = True

            if f_node in backward_visited:
                meeting_point = f_node
                break

            for neighbor in grid.get_neighbors(*f_node):
                new_dist = forward_dist[f_node] + 1
                if neighbor not in forward_dist or new_dist < forward_dist[neighbor]:
                    forward_dist[neighbor] = new_dist
                    heapq.heappush(forward_queue, (new_dist, neighbor))
                    forward_prev[neighbor] = f_node

        # Backward step
        if backward_queue:
            b_dist, b_node = heapq.heappop(backward_queue)
            if b_node in backward_visited:
                continue
            backward_visited[b_node] = True

            if b_node in forward_visited:
                meeting_point = b_node
                break

            for neighbor in grid.get_neighbors(*b_node):
                new_dist = backward_dist[b_node] + 1
                if neighbor not in backward_dist or new_dist < backward_dist[neighbor]:
                    backward_dist[neighbor] = new_dist
                    heapq.heappush(backward_queue, (new_dist, neighbor))
                    backward_prev[neighbor] = b_node

    # If a path was found, reconstruct it
    if meeting_point:
        path = []

        # From meeting point to start
        node = meeting_point
        while node in forward_prev:
            path.append(node)
            node = forward_prev[node]
        path.append(start_pos)
        path.reverse()

        # From meeting point to end
        node = meeting_point
        while node in backward_prev:
            node = backward_prev[node]
            path.append(node)

        grid.draw_path(path)
    else:
        print("No path found.")
