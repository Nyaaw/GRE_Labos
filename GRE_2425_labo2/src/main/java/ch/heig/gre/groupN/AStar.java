package ch.heig.gre.groupN;

import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.GridMazeSolver;

import java.util.*;

/**
 * AStar implémente l'algorithme A* pour trouver le plus court chemin entre deux sommets
 * dans un graphe tout en prenant en compte les 5 heuristiques différentes proposées
 *
 * @author Ripoll Pierric - Sottile Alan
 */

public final class AStar implements GridMazeSolver {
    private static final int NULL = -1;

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

    /**
     * Enumération des heuristiques disponibles
     */
    public enum Heuristic {
        /**
         * Heuristique de Dijkstra, qui ne prend pas en compte la distance restante
         */
        DIJKSTRA,

        /**
         * Heuristique de l'infini, qui renvoie la distance infinie entre le sommet source et le sommet destination.
         */
        INFINITY_NORM {
            @Override
            public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan) {
                coords coord = new coords(source, destination, graphWidth);
                return Math.max(Math.abs(coord.x1 - coord.x2), Math.abs(coord.y1 - coord.y2));
            }
        },

        /**
         * Heuristique de la norme euclidienne qui renvoie la distance euclidienne
         */
        EUCLIDEAN_NORM {
            @Override
            public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan) {
                coords coord = new coords(source, destination, graphWidth);
                return Math.sqrt(Math.pow(coord.x1 - coord.x2, 2) + Math.pow(coord.y1 - coord.y2, 2));
            }
        },

        /**
         * Heuristique de la distance de Manhattan
         */
        MANHATTAN {
            @Override
            public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan) {
                coords coord = new coords(source, destination, graphWidth);
                return Math.abs(coord.x1 - coord.x2) + Math.abs(coord.y1 - coord.y2);
            }
        },

        /**
         * Heuristique de la distance de Manhattan multipliée par kManhattan.
         */
        K_MANHATTAN {
            @Override
            public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan) {
                coords coord = new coords(source, destination, graphWidth);
                return kManhattan * (Math.abs(coord.x1 - coord.x2) + Math.abs(coord.y1 - coord.y2));
            }
        };

        /**
         * Coordonnées bidimensionnelles d'un sommet dans le graphe.
         */
        private record coords(int x1, int y1, int x2, int y2) {
            public coords(int source, int destination, int graphWidth) {
                this(source % graphWidth, source / graphWidth, destination % graphWidth, destination / graphWidth);
            }
        }

        /**
         * Calcule la valeur de l'heuristique entre deux sommets.
         */
        public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan) {
            return 0;
        }
    }

    //Retourne le plus court chemin du sommet source au sommet destination en utilisant l'algorithme A*
    @Override
    public Result solve(GridGraph2D grid,
                        PositiveWeightFunction weights,
                        int source,
                        int destination,
                        VertexLabelling<Boolean> processed) {

        int n = grid.width() * grid.height(); // Le nombre de sommets
        int[] distance = new int[n]; // La distance du sommet source à chaque sommet
        int[] predecessors = new int[n]; // Les prédécesseurs de chaque sommet
        int treated = 0; // Utilisé pour suivre le nombre de sommets traités

        // Initialisation des tableaux
        for (int i = 0; i < n; i++) {
            distance[i] = Integer.MAX_VALUE;
            predecessors[i] = NULL;
        }
        distance[source] = 0;

        // Initialisation de la file de priorité
        PriorityQueue<Integer> queue = new PriorityQueue<>(
                Comparator.comparingDouble(v ->
                        distance[v] + heuristic.computeHeuristic(v, destination, grid.width(), kManhattan))
        );
        queue.offer(source);

        while (!queue.isEmpty()) {
            // On traite le sommet avec la plus faible distance en tenant compte de l'heuristique
            int currentVertex = queue.poll();
            treated++;

            if (processed != null) {
                processed.setLabel(currentVertex, true);
            }

            if (currentVertex == destination) {
                break;
            }

            // On met à jour la distance de chaque voisin en prenant en compte l'heuristique
            for (int neighbor : grid.neighbors(currentVertex)) {
                int edgeWeight = weights.get(currentVertex, neighbor);
                // Si un changement est nécessaire dans le tableau, on le met à jour et l'ajoute à la file de priorité
                if (distance[neighbor] > distance[currentVertex] + edgeWeight) {
                    distance[neighbor] = distance[currentVertex] + edgeWeight;
                    predecessors[neighbor] = currentVertex;
                    queue.offer(neighbor);
                }
            }
        }

        // On crée le chemin inversé car il est plus facile de le faire ainsi
        List<Integer> path = new ArrayList<>();
        for (int v = destination; v != NULL; v = predecessors[v]) {
            path.add(v);
        }

        // On inverse le chemin ici, car on a vu que c'est O(1) au lieu de O(n) si on utilisait Collections.reverse()
        return new Result(path.reversed(), distance[destination], treated);
    }
}
