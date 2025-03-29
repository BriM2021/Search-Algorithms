import numpy as np
import pickle
from collections import deque
import math

# General Notes:
# - Update the provided file name (code_<RollNumber>.py) as per the instructions.
# - Do not change the function name, number of parameters or the sequence of parameters.
# - The expected output for each function is a path (list of node names)
# - Ensure that the returned path includes both the start node and the goal node, in the correct order.
# - If no valid path exists between the start and goal nodes, the function should return None.


# Algorithm: Iterative Deepening Search (IDS)

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 2, 9, 8, 5, 97, 98, 12]

def get_ids_path(adj_matrix, start_node, goal_node):
  # added for cases where no solution
  def check(adj_matrix, start_node, goal_node):
    q = deque([start_node])
    vis = set([start_node]) 

    while q:
      u = q.popleft()

      if u == goal_node:
        return True  

      for v in range(len(adj_matrix[u])):
        if adj_matrix[u][v] and v not in vis:
          vis.add(v)
          q.append(v)
    return False
  
  def depth_limited_search(adj_matrix, start_node, goal_node, limit):
    frontier = [(start_node, [start_node])]  
    result = None
    
    while frontier:
      u, path = frontier.pop()
      if u == goal_node:
        return path 
        
      if len(path) > limit:
        result = 'cutoff'
      else:
        for v in reversed(range(len(adj_matrix[u]))):
          # print(neighbor)
          if adj_matrix[u][v] and v not in path: 
            new_path = path + [v]
            frontier.append((v, new_path))
            # print(new_path)
    return result

  if(check(adj_matrix, start_node, goal_node)):
    for depth in range(len(adj_matrix)):
      # print("Plsss work", depth)
      result = depth_limited_search(adj_matrix, start_node, goal_node, depth)
      if result != 'cutoff':  
        return result
      # print(depth)
  else:
    return None


# Algorithm: Bi-Directional Search

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 2, 9, 8, 5, 97, 98, 12]

def get_bidirectional_search_path(adj_matrix, start_node, goal_node):
  def construct_path(vis_s, vis_g, meeting_point):
    meet= meeting_point
    path1 = []
    while meeting_point is not None:
        path1.append(meeting_point)
        meeting_point = vis_s[meeting_point]
    
    path1.reverse()  
    path2 = []
    meeting_point = vis_g[meet]
    while meeting_point is not None:
        path2.append(meeting_point)
        meeting_point = vis_g[meeting_point]
    return path1 + path2

  n = len(adj_matrix)
  if start_node == goal_node:
    return [start_node]
  
  q_start = deque([start_node])
  q_goal = deque([goal_node])
  
  visited_start = {start_node: None}
  visited_goal = {goal_node: None}

  while q_start and q_goal:
    q1=[]
    q2=[]
    while q_start:
      u_start = q_start.popleft()
      for v in range(n):
        if adj_matrix[u_start][v] and v not in visited_start:
          # print(u_start, "jhs", v)
          visited_start[v] = u_start
          q1.append(v)
          if v in visited_goal:  
            # print(v)
            return construct_path(visited_start, visited_goal, v)
    for i in q1:
      q_start.append(i)

    while q_goal:
      u_goal = q_goal.popleft()
      for v in range(n):
        if adj_matrix[v][u_goal] and v not in visited_goal:
          # print(u_goal, "abs", v)
          visited_goal[v] = u_goal
          q2.append(v)
          if v in visited_start: 
            # print(v)
            return construct_path(visited_start, visited_goal, v)
    for i in q2:
      q_goal.append(i)

  return None 

# Algorithm: A* Search Algorithm

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - node_attributes: Dictionary of node attributes containing x, y coordinates for heuristic calculations.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 28, 10, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 27, 9, 8, 5, 97, 28, 10, 12]

def get_astar_search_path(adj_matrix, node_attributes, start_node, goal_node):
  def euclidean_dist(node1, node2, node_attributes):
    x1, y1 = node_attributes[node1]['x'], node_attributes[node1]['y']
    x2, y2 = node_attributes[node2]['x'], node_attributes[node2]['y']
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

  def h_n(node, start_node, goal_node, node_attributes):
    return euclidean_dist(start_node, node, node_attributes) + euclidean_dist(node, goal_node, node_attributes)


  open_set = set([start_node])
  closed_set = set()

  g = {}  
  parents = {}  

  g[start_node] = 0
  parents[start_node] = start_node

  while len(open_set) > 0:
    node = None
    n= adj_matrix[node]
    for v in open_set:
      # print(v)
      if node is None:
        node = v
      elif g[v] + h_n(v, start_node, goal_node, node_attributes) < g[node] + h_n(node, start_node, goal_node, node_attributes):
        node = v
      elif g[v] + h_n(v, start_node, goal_node, node_attributes) == g[node] + h_n(node, start_node, goal_node, node_attributes) and v < n:
        node = v
    
    # print(node)
    if node == goal_node:
      path = []

      while parents[node] != node:
        path.append(node)
        node = parents[node]

      path.append(start_node)
      path.reverse()
      return path

    neighbors = []
    for i, weight in enumerate(adj_matrix[node]):
      if weight > 0:
        neighbors.append((i, weight))
  
    for (neighbor, weight) in neighbors:
      if neighbor not in open_set and neighbor not in closed_set:
        open_set.add(neighbor)
        parents[neighbor] = node
        g[neighbor] = g[node] + weight
      else:
        if g[neighbor] > g[node] + weight:
          g[neighbor] = g[node] + weight
          parents[neighbor] = node

          if neighbor in closed_set:
              closed_set.remove(neighbor)
              open_set.add(neighbor)

    open_set.remove(node)
    closed_set.add(node)

  return None


# Algorithm: Bi-Directional Heuristic Search

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - node_attributes: Dictionary of node attributes containing x, y coordinates for heuristic calculations.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 34, 33, 11, 32, 31, 3, 5, 97, 28, 10, 12]

def get_bidirectional_heuristic_search_path(adj_matrix, node_attributes, start_node, goal_node):
  def euclidean_dist(node1, node2, node_attributes):
    x1, y1 = node_attributes[node1]['x'], node_attributes[node1]['y']
    x2, y2 = node_attributes[node2]['x'], node_attributes[node2]['y']
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

  def h_n(node, start_node, goal_node, node_attributes):
    return euclidean_dist(start_node, node, node_attributes) + euclidean_dist(node, goal_node, node_attributes)

  def reconstruct_path(parents_start, parents_goal, meeting_point):
    meet= meeting_point
    path1 = []
    while meeting_point is not None:
      path1.append(meeting_point)
      meeting_point = parents_start.get(meeting_point)
    
    path1.reverse()  
    meeting_point=meet
    # print(meeting_point)
    path2 = []
    while meeting_point is not None:
      path2.append(meeting_point)
      meeting_point = parents_goal.get(meeting_point)
    # print(path_end)
    return path1 + path2[1:]  
  
  if start_node == goal_node:
    return [start_node]
  num_nodes = len(adj_matrix)
  reverse_adj_matrix = np.zeros((num_nodes, num_nodes))
  
  for i in range(num_nodes):
    for j in range(num_nodes):
      if adj_matrix[i][j] > 0:  
        reverse_adj_matrix[j][i] = adj_matrix[i][j]  

  open_set_start = {start_node}
  open_set_goal = {goal_node}
  
  g_score_start = {start_node: 0}
  g_score_goal = {goal_node: 0}
  
  parents_start = {start_node: None}
  parents_goal = {goal_node: None}

  while open_set_start and open_set_goal:
    u_start = min(open_set_start, key=lambda n: g_score_start[n] + h_n(n, start_node, goal_node, node_attributes))

    if u_start in open_set_goal:
      return reconstruct_path(parents_start, parents_goal, u_start)

    open_set_start.remove(u_start)

    for neighbor in range(len(adj_matrix[u_start])):
      if adj_matrix[u_start][neighbor] > 0:  
        # print("plss chal ja, meko cry aa rha")
        temp_g_n = g_score_start[u_start] + adj_matrix[u_start][neighbor]
        
        if neighbor not in g_score_start or temp_g_n < g_score_start[neighbor]:
          g_score_start[neighbor] = temp_g_n
          parents_start[neighbor] = u_start
          
          if neighbor not in open_set_start:
            open_set_start.add(neighbor)

    current_goal = min(open_set_goal, key=lambda n: g_score_goal[n] + h_n(n, start_node,goal_node, node_attributes))

    if current_goal in open_set_start:
      return reconstruct_path(parents_start, parents_goal, current_goal)

    open_set_goal.remove(current_goal)

    for neighbor in range(len(reverse_adj_matrix[current_goal])):
      if reverse_adj_matrix[current_goal][neighbor] > 0:  
        # print(neighbor)
        # print("plss chal ja")
        tentative_g_score = g_score_goal[current_goal] + reverse_adj_matrix[current_goal][neighbor]
        
        if neighbor not in g_score_goal or tentative_g_score < g_score_goal[neighbor]:
          g_score_goal[neighbor] = tentative_g_score
          parents_goal[neighbor] = current_goal
          
          if neighbor not in open_set_goal:
            open_set_goal.add(neighbor)
  return None


# Bonus Problem
 
# Input:
# - adj_matrix: A 2D list or numpy array representing the adjacency matrix of the graph.

# Return:
# - A list of tuples where each tuple (u, v) represents an edge between nodes u and v.
#   These are the vulnerable roads whose removal would disconnect parts of the graph.

# Note:
# - The graph is undirected, so if an edge (u, v) is vulnerable, then (v, u) should not be repeated in the output list.
# - If the input graph has no vulnerable roads, return an empty list [].

def bonus_problem(adj_matrix):
  n = len(adj_matrix)
  # # Converting directed graph to undirected
  # for i in range(n):
  #   for j in range(n):
  #     if adj_matrix[i][j]:
  #       adj_matrix[i][j] = 1
  #       adj_matrix[j][i] = 1

  discovery_time = [-1] * n  
  low = [-1] * n             
  parent = [-1] * n         
  answer = []               
  time = [0]

  def dfs(u):
    # print(u)
    discovery_time[u] = low[u] = time[0]
    time[0] += 1
    for v in range(n):
      if adj_matrix[u][v] !=0 :  
        if discovery_time[v] == -1: 
          parent[v] = u
          dfs(v)
          low[u] = min(low[u], low[v])
          # print(v)
          if low[v] > discovery_time[u]:
            answer.append((u, v))
        elif v != parent[u]:  
          low[u] = min(low[u], discovery_time[v])

  for i in range(n):
    if discovery_time[i] == -1:
      dfs(i)
  return answer


if __name__ == "__main__":
  adj_matrix = np.load('IIIT_Delhi.npy')
  with open('IIIT_Delhi.pkl', 'rb') as f:
    node_attributes = pickle.load(f)

  start_node = int(input("Enter the start node: "))
  end_node = int(input("Enter the end node: "))

  print(f'Iterative Deepening Search Path: {get_ids_path(adj_matrix,start_node,end_node)}')
  print(f'Bidirectional Search Path: {get_bidirectional_search_path(adj_matrix,start_node,end_node)}')
  print(f'A* Path: {get_astar_search_path(adj_matrix,node_attributes,start_node,end_node)}')
  print(f'Bidirectional Heuristic Search Path: {get_bidirectional_heuristic_search_path(adj_matrix,node_attributes,start_node,end_node)}')
  print(f'Bonus Problem: {bonus_problem(adj_matrix)}')