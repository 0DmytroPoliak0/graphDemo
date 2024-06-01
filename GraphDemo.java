package org.example;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// The Edge class represents an edge in a graph. It stores the indices (u and v) of the two vertices
// that the edge connects, as well as the weight of the edge.
class Edge {
    int u; // The index of the first vertex in the edge
    int v; // The index of the second vertex in the edge
    int weight; // The weight of the edge

    // Constructor for the Edge class
    public Edge(int u, int v, int weight) {
        this.u = u;
        this.v = v;
        this.weight = weight;
    }
}

// The Graph class represents a weighted graph using a list of vertices and an adjacency list
// to store edges. The vertices are stored in a generic list to allow different types of vertex identifiers.
class Graph<V> {
    private List<V> vertices; // List of vertices in the graph
    private Map<V, List<Edge>> adjacencyEdges; // Adjacency list to store edges

    // Constructor for the Graph class
    public Graph() {
        this.vertices = new ArrayList<>();
        this.adjacencyEdges = new HashMap<>();
    }

    // Adds a vertex to the graph. The vertex is added to the list of vertices and a new
    // empty list of edges is created in the adjacency list for that vertex.
    public void addVertex(V vertex) {
        vertices.add(vertex);
        adjacencyEdges.put(vertex, new ArrayList<>());
    }

    // Adds an edge to the graph. The edge is added to the adjacency lists of both vertices
    // that it connects. This is for an undirected graph, so we add the edge in both directions.
    public void addEdge(Edge e) {
        adjacencyEdges.get(vertices.get(e.u)).add(e);
        adjacencyEdges.get(vertices.get(e.v)).add(new Edge(e.v, e.u, e.weight));
    }

    // Removes a vertex from the graph. It removes the vertex from the list of vertices,
    // and also removes all edges connected to it from the adjacency list. It then updates
    // the remaining edges to reflect the changed indices.
    public void removeVertex(V vertex) {
        int index = vertices.indexOf(vertex);
        if (index == -1) {
            return; // Vertex does not exist
        }
        vertices.remove(index);
        adjacencyEdges.remove(vertex);
        for (List<Edge> edges : adjacencyEdges.values()) {
            edges.removeIf(edge -> edge.u == index || edge.v == index);
        }
        // After removing the vertex, we need to update the indices of the vertices in the edges
        for (V key : adjacencyEdges.keySet()) {
            List<Edge> updatedList = new ArrayList<>();
            for (Edge edge : adjacencyEdges.get(key)) {
                if (edge.u > index) edge.u--;
                if (edge.v > index) edge.v--;
                updatedList.add(edge);
            }
            adjacencyEdges.put(key, updatedList);
        }
    }

    // Removes an edge from the graph by removing it from the adjacency lists of both vertices
    // that it connects.
    public void removeEdge(Edge e) {
        adjacencyEdges.get(vertices.get(e.u)).removeIf(edge -> edge.u == e.u && edge.v == e.v && edge.weight == e.weight);
        adjacencyEdges.get(vertices.get(e.v)).removeIf(edge -> edge.u == e.v && edge.v == e.u && edge.weight == e.weight);
    }

    // Initiates a depth-first search (DFS) starting from a given start vertex. It prints the vertices
    // visited in the order they were encountered.
    public void dfs(V startVertex) {
        boolean[] visited = new boolean[vertices.size()];
        int startIndex = vertices.indexOf(startVertex);
        if (startIndex != -1) {
            dfsHelper(startIndex, visited);
        }
        System.out.println();
    }

    // Helper method for DFS. It recursively visits vertices in depth-first order.
    private void dfsHelper(int index, boolean[] visited) {
        visited[index] = true;
        System.out.print(vertices.get(index) + " ");

        for (Edge edge : adjacencyEdges.get(vertices.get(index))) {
            if (!visited[edge.v]) {
                dfsHelper(edge.v, visited);
            }
        }
    }
}

// The GraphDemo class contains the main method which demonstrates the creation of a graph,
// performing DFS, removing a vertex, and then performing DFS again.
public class GraphDemo {
    public static void main(String[] args) {
        Graph<Integer> graph = new Graph<>();
        // Initialize the graph by adding vertices
        for (int i = 0; i <= 4; i++) {
            graph.addVertex(i);
        }
        // Add edges to the graph based on the provided image
        graph.addEdge(new Edge(0, 1, 2));
        graph.addEdge(new Edge(0, 3, 3));
        graph.addEdge(new Edge(1, 3, 8));
        graph.addEdge(new Edge(1, 2, 7));
        graph.addEdge(new Edge(3, 4, 4));
        graph.addEdge(new Edge(2, 4, 6));
        graph.addEdge(new Edge(2, 3, 3));
        graph.addEdge(new Edge(3, 2, 4));
        graph.addEdge(new Edge(4, 3, 5));

        // Perform depth-first search starting from vertex 0
        System.out.println("DFS starting from vertex 0:");
        graph.dfs(0);

        // Remove vertex 4 and perform depth-first search again
        graph.removeVertex(4);
        System.out.println("DFS after removing vertex 4, starting from vertex 0:");
        graph.dfs(0);
    }
}
