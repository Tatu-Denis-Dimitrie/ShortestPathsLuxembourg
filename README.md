# Graph Visualization in Qt with C++

## Overview
This project is a **graph visualization tool** built with **Qt and C++**, allowing users to load a graph from an XML file, display nodes and edges, and find the shortest path between selected nodes using **Dijkstra's algorithm**.

## Features
✔ **Graph rendering** with nodes and edges drawn dynamically.  
✔ **Shortest path calculation** using Dijkstra's algorithm.  
✔ **XML-based graph loading**, supporting large datasets.  
✔ **Interactive node selection** via mouse clicks.  
✔ **Graph connectivity check** to verify if the graph is fully connected.  
✔ **Adaptive scaling** to fit the graph within the window.  
✔ **Customizable edge costs** and adjacency list representation.

## File Structure
- **Graph.h / Graph.cpp** - Core logic for graph rendering, file parsing, and shortest path search.  
- **Harta_Luxemburg.xml** - Sample dataset containing nodes and edges.  
- **MainWindow.h / MainWindow.cpp** - Manages the main UI.  

## Functionality
### 1. Graph Rendering
- **Nodes** are displayed as small circles.
- **Edges** are drawn between connected nodes.
- **Selected nodes** are highlighted in red.
- **Shortest path** is drawn in blue.

### 2. File Parsing
The graph is loaded from an XML file (**Harta_Luxemburg.xml**) containing:
- Nodes with latitude and longitude attributes.
- Edges connecting nodes, with associated travel costs.

### 3. Shortest Path Computation
- Uses **Dijkstra's algorithm** with a priority queue.
- Returns the shortest path between two selected nodes.
- Handles disconnected components gracefully.

### 4. Interactive Features
- **Mouse click on a node**: Selects it.
- **Selecting two nodes**: Triggers the shortest path computation.
- **Console logs**: Display relevant debugging information.

## How to Run
1. Open the project in **Qt Creator**.
2. Ensure that Qt is properly installed.
3. Compile and run the project.
4. Click on nodes to select them and compute the shortest path.

## Dependencies
- **Qt Framework** (QtWidgets, QtGui, QtXml)
- **C++ Standard Library**

## Future Enhancements
- Add support for weighted graphs with different visualization styles.
- Implement additional pathfinding algorithms like **A\* search**.
- Introduce **graph editing features** (add/remove nodes and edges dynamically).

This project is a powerful tool for understanding graph structures and algorithms. 🚀
