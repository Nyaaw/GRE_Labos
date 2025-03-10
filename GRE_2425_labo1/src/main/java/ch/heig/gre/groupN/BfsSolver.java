package ch.heig.gre.groupN;

import ch.heig.gre.graph.Graph;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.MazeSolver;

import java.util.*;

/**
 * BfsSolver implements the BFS algorithm to find the shortest path between two vertices in a graph.
 *
 * @author Ripoll Pierric - Sottile Alan
 */
public final class BfsSolver implements MazeSolver {
    @Override
    public List<Integer> solve(Graph graph, int source, int destination, VertexLabelling<Integer> treatments) {
        //number of vertices in the graph
        int n = graph.nbVertices();

        // 2 tables to store the distance to the source and predecessor of each vertex
        int[] distance = new int[n];
        int[] predecessor = new int[n];

        // Initializing the BFS tables, we initialize them to -1 to indicate we haven't visited them yet
        for (int i = 0; i < n; i++) {
            distance[i] = -1;
            predecessor[i] = -1;
        }

        // The source has a distance of 0 from itself
        distance[source] = 0;

        // Initializing the queue
        Queue<Integer> queue = new LinkedList<>();
        queue.add(source);


        // BFS algorithm
        while (!queue.isEmpty()) {
            int current = queue.poll();

            if (treatments != null) {
                treatments.setLabel(current, distance[current]);
            }

            // if we are at the destination, there is no need to continue treatment
            if (current == destination) {
                break;
            }

            //for each unvisited neighbors of the current vertex,
            // we update their distance to current +1, and we put
            // current as their predecessor.
            for (int neighbor : graph.neighbors(current)) {
                if (distance[neighbor] == -1) {
                    distance[neighbor] = distance[current] + 1;
                    predecessor[neighbor] = current;
                    //We add them to the queue to treat them next
                    queue.add(neighbor);
                }
            }
        }

        // Constructing reversed path (from dest to source)
        List<Integer> path = new ArrayList<>();
        for (int v = destination; v != -1; v = predecessor[v]) {
            path.add(v);
        }

        //We need to reverse it because we filled it from the destination to the source
        Collections.reverse(path);

        return path;
    }
}

