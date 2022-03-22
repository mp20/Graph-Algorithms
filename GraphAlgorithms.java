import java.util.List;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.HashSet;
import java.util.Set;
import java.util.Map;
import java.util.Queue;
import java.util.PriorityQueue;
import java.util.HashMap;
/**
 * Your implementation of various different graph algorithms.
 *
 * @author Ariya Nazari Foroshani
 * @version 1.0
 * @userid aforoshani3
 * @GTID YOUR GT ID HERE 903627990
 *
 * Collaborators: None
 *
 * Resources: None
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * You may import/use java.util.Set, java.util.List, java.util.Queue, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("start or graph is null");
        } else if (!graph.getAdjList().containsKey(start)) {
            throw new IllegalArgumentException("start is not in the graph");
        }
        Set<Vertex<T>> visitedSet = new HashSet<>();
        Queue<Vertex<T>> queue = new LinkedList<>();
        //mark the start index as visited
        visitedSet.add(start);
        //add start index to the queue
        queue.add(start);
        List<Vertex<T>> list = new ArrayList<>();
        while (!queue.isEmpty()) {
            Vertex<T> v = queue.remove();
            list.add(v);
            for (VertexDistance<T> w : getAdj(graph, v)) {
                //make sure v's adjacent vertices are not visited
                if (!visitedSet.contains(w.getVertex())) {
                    //mark w as visited and add them to the queue
                    queue.add(w.getVertex());
                    visitedSet.add(w.getVertex());
                }
            }
        }
        return list;
    }

    /**
     *
     * @param graph the graph to search through
     * @param v The vertex that we want to find the neighbors of
     * @param <T> the generic type of data
     * @return the list of adjacent vertices to the passed in vertex
     */
    private static <T> List<VertexDistance<T>> getAdj(Graph<T> graph, Vertex<T> v) {
        return graph.getAdjList().get(v);
    }


    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * all points for this method.
     *
     * You may import/use java.util.Set, java.util.List, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (graph == null || start == null) {
            throw new IllegalArgumentException("Graph/start is null");
        } else if (!graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("Start does not exist in the graph");
        }
        List<Vertex<T>> list = new ArrayList<>();
        Set<Vertex<T>> visitedSet = new HashSet<>();
        dfsHelper(start, graph, visitedSet, list);
        return list;
    }

    /**
     *
     * @param curr the current vertex
     * @param graph the graph to search through
     * @param visitedSet The set of all visited verticies
     * @param list list of the verticies
     * @param <T> the generic type of data
     */
    private static <T> void dfsHelper(Vertex<T> curr, Graph<T> graph, Set<Vertex<T>> visitedSet,
                                       List<Vertex<T>> list) {
        //start by marking curr as visited
        list.add(curr);
        visitedSet.add(curr);
        for (VertexDistance<T> w : getAdj(graph, curr)) {
            if (!visitedSet.contains(w.getVertex())) {
                dfsHelper(w.getVertex(), graph, visitedSet, list);
            }
        }
    }


    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing
     * infinity) if no path exists.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Map, and java.util.Set and any class that
     * implements the aforementioned interfaces, as long as your use of it
     * is efficient as possible.
     *
     * You should implement the version of Dijkstra's where you use two
     * termination conditions in conjunction.
     *
     * 1) Check if all of the vertices have been visited.
     * 2) Check if the PQ is empty yet.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the Dijkstra's on (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every
     * other node in the graph
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start, Graph<T> graph) {
        if (graph == null || start == null) {
            throw new IllegalArgumentException("Graph/start is null");
        } else if (!graph.getVertices().contains(start)) {
            throw new IllegalArgumentException("Start does not exist in the graph");
        }
        Set<Vertex<T>> visitedSet = new HashSet<>();
        Map<Vertex<T>, Integer> distance = new HashMap<>();
        Queue<VertexDistance<T>> queue = new PriorityQueue<>();
        for (Vertex<T> vertex : graph.getAdjList().keySet()) {
            if (vertex.equals(start)) {
                //the start always is 0
                distance.put(vertex, 0);
            } else {
                //set it to infinity because we dont know the distance yet
                distance.put(vertex, Integer.MAX_VALUE);
            }
        }
        //add source vertex with distance 0
        VertexDistance<T> source = new VertexDistance<>(start, 0);
        queue.add(source);
        while (!queue.isEmpty() && visitedSet.size() < graph.getAdjList().size()) {
            VertexDistance<T> current = queue.remove();
            Vertex<T> currentVertex = current.getVertex();
            if (!visitedSet.contains(currentVertex)) {
                visitedSet.add(currentVertex);
                distance.put(currentVertex, current.getDistance());
                for (VertexDistance<T> adjVertex : getAdj(graph, currentVertex)) {
                    if (!visitedSet.contains(adjVertex)) {
                        int shortest = adjVertex.getDistance() + current.getDistance();
                        VertexDistance<T> w = new VertexDistance<>(adjVertex.getVertex(), shortest);
                        queue.add(w);
                    }
                }
            }
        }
        return distance;
    }

    /**
     * Runs Kruskal's algorithm on the given graph and returns the Minimal
     * Spanning Tree (MST) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid MST exists, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * reverse edge to the set as well. This is for testing purposes. This
     * reverse edge does not need to be the one from the graph itself; you can
     * just make a new edge object representing the reverse edge.
     *
     * You may assume that there will only be one valid MST that can be formed.
     *
     * Kruskal's will also require you to use a Disjoint Set which has been
     * provided for you. A Disjoint Set will keep track of which vertices are
     * connected given the edges in your current MST, allowing you to easily
     * figure out whether adding an edge will create a cycle. Refer
     * to the DisjointSet and DisjointSetNode classes that
     * have been provided to you for more information.
     *
     * You should NOT allow self-loops or parallel edges into the MST.
     *
     * By using the Disjoint Set provided, you can avoid adding self-loops and
     * parallel edges into the MST.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Set, and any class that implements the aforementioned
     * interfaces.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param graph the graph we are applying Kruskals to
     * @return the MST of the graph or null if there is no valid MST
     * @throws IllegalArgumentException if any input is null
     */
    public static <T> Set<Edge<T>> kruskals(Graph<T> graph) {
        if (graph == null) {
            throw new IllegalArgumentException("Cannot use null graph");
        }
        int edges = graph.getEdges().size();
        int vertices = graph.getVertices().size();
        //edges must be vertices -1
        DisjointSet<Vertex<T>> disjointSet = new DisjointSet<>();
        Set<Edge<T>> mst = new HashSet<>();
        PriorityQueue<Edge<T>> queue = new PriorityQueue<>(graph.getEdges());
        while (!queue.isEmpty() && mst.size() < 2 * (vertices - 1)) {
            Edge<T> edge = queue.remove();
            Vertex<T> u = edge.getU();
            Vertex<T> v = edge.getV();
            if (!disjointSet.find(u).equals(disjointSet.find(v))) {
                mst.add(edge);
                disjointSet.union(disjointSet.find(u), disjointSet.find(v));
                //add the opposite because it is undirected
                Edge<T> reverse = new Edge<>(edge.getV(), edge.getU(), edge.getWeight());
                mst.add(reverse);
            }
        }
        if (mst.size() != 2 * (vertices - 1)) {
            return null;
        }
        return mst;
    }
}
