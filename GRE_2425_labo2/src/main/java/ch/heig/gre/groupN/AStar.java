package ch.heig.gre.groupN;

import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.GridMazeSolver;

import java.util.*;

/**
 * AStar implements the A* algorithm to find the shortest path between two vertices
 * in a graph while taking in account the 5 different heuristics we offer
 *
 * @author Ripoll Pierric - Sottile Alan
 */

public final class AStar implements GridMazeSolver {

    private static final int NULL = -1;

    public enum Heuristic {
        DIJKSTRA,

        INFINITY_NORM {
            @Override
            public int computeHeuristic(int source, int destination, int k) {
                return 0;
            }
        },

        EUCLIDEAN_NORM {
            @Override
            public int computeHeuristic(int source, int destination, int k) {
                return 0;
            }
        },

        MANHATTAN {
            @Override
            public int computeHeuristic(int source, int destination, int k) {
                return 0;
            }
        },

        K_MANHATTAN {
            @Override
            public int computeHeuristic(int source, int destination, int k) {
                return 0;
            }
        };

        public int computeHeuristic(int source, int destination, int k) {
            return 0;
        }


    }

    /**
     * Heuristique utilisée pour l'algorithme A*.
     */
    private final Heuristic heuristic;

    /**
     * Facteur multiplicatif de la distance de Manhattan utilisé par l'heuristique K-Manhattan.
     */
    private final int kManhattan;

    public AStar(Heuristic heuristic) {
        this(heuristic, 1);
    }

    public AStar(Heuristic heuristic, int kManhattan) {
        this.heuristic = heuristic;
        this.kManhattan = kManhattan;
    }


    //Returns the shortest path from the source to the destination while using the A* algorithm
    @Override
    public Result solve(GridGraph2D grid,
                        PositiveWeightFunction weights,
                        int source,
                        int destination,
                        VertexLabelling<Boolean> processed) {


        int n = grid.width() * grid.height(); //The amount of vertices
        int[] distance = new int[n]; //The distance from the source to each vertex
        int[] predecessors = new int[n]; //The predecessors of each vertex
        int treated = 0; //Used to keep track of the amount of treated vertices

        //Initializing the tables
        for (int i = 0; i < n; i++) {
            distance[i] = Integer.MAX_VALUE;
            predecessors[i] = NULL;
        }
        distance[source] = 0;

        //Initializing the priority queue
        PriorityQueue<Integer> queue = new PriorityQueue<>(
                Comparator.comparingInt(v -> {
                    if (heuristic == Heuristic.K_MANHATTAN) {
                        return distance[v] + heuristic.computeHeuristic(v, destination, kManhattan);
                    } else {
                        return distance[v] + heuristic.computeHeuristic(v, destination, 1);
                    }
                })
        );
        queue.offer(source);

        while (!queue.isEmpty()) {
            //We treat the vertex with the lowest distance while taking in account the heuristic
            int currentVertex = queue.poll();
            treated++;

            if (processed != null) {
                processed.setLabel(currentVertex, true);
            }

            if (currentVertex == destination) {
                break;
            }

            //We update the distance of each neighbors by taking into account the heuristic
            for (int neighbor : grid.neighbors(currentVertex)) {
                int edgeWeight = weights.get(currentVertex, neighbor);
                //If a change is needed in the table, we update it and add it to the queue
                if (distance[neighbor] > distance[currentVertex] + edgeWeight || distance[neighbor] == Integer.MAX_VALUE) {
                    distance[neighbor] = distance[currentVertex] + edgeWeight;
                    predecessors[neighbor] = currentVertex;
                    queue.offer(neighbor);
                }
            }
        }

        //We create the reversed path because it's easier to do so
        List<Integer> path = new ArrayList<>();
        for (int v = destination; v != -1; v = predecessors[v]) {
            path.add(v);
        }

        //We reverse the path here, since we saw it's O(1) instead of O(n) if we used Collections.reverse()
        return new Result(path.reversed(), distance[destination], treated);
    }
}