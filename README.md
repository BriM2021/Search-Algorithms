# Search Algorithms

## Overview

This project implements various pathfinding algorithms on a geographical graph representing an institute's road network. The graph is given as an adjacency matrix where nodes represent geographical locations, and edges represent roads connecting those locations. The primary objective is to find the shortest path between a source and destination node using both uninformed and informed search algorithms.

<img width="419" alt="image" src="https://github.com/user-attachments/assets/f4945177-7a91-426b-8fa8-4ba1f362bf35" />    

Figure 1: A graph representing IIIT Delhi   


The project includes:

- Implementation of **Uninformed Search Algorithms**:
  - Iterative Deepening Search
  - Bidirectional Breadth-First Search

- Implementation of **Informed Search Algorithms**:
  - A* Search
  - Bidirectional A* Search


- Identification of vulnerable roads in the geographical network whose removal would disconnect components of the graph.

---

## Algorithms Implemented

### 1. **Uninformed Search Algorithms**
   - **Iterative Deepening Search**: Combines the benefits of depth-first and breadth-first search.
   - **Bidirectional Breadth-First Search**: Executes two breadth-first searches, one from the source and one from the destination, to meet in the middle.

### 2. **Informed Search Algorithms**
   - **A* Search**: An informed search algorithm that uses a heuristic to estimate the cost of the path from the current node to the destination.
   - **Bidirectional A* Search**: A combination of A* Search and bidirectional search for faster pathfinding.

### 3. **Vulnerable Road Identification **
   - Identifies vulnerable edges (roads) in the geographical network, whose removal would increase the number of disconnected components in the graph.

---

## Getting Started

### Prerequisites

To run this project, you will need:
- Python 3.x
- Libraries: `numpy`, `matplotlib`, `networkx` (for graph visualization and operations)

You can install the required dependencies by running:

```bash
pip install -r requirements.txt
