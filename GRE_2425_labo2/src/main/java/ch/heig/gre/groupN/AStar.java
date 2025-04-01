package ch.heig.gre.groupN;

import ch.heig.gre.graph.GridGraph2D;
import ch.heig.gre.graph.PositiveWeightFunction;
import ch.heig.gre.graph.VertexLabelling;
import ch.heig.gre.maze.GridMazeSolver;

public final class AStar implements GridMazeSolver {

  public enum Heuristic {

    DIJKSTRA,
    
    INFINITY_NORM{
      @Override
      public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan){
        coords coord = new coords(source, destination, graphWidth);

        return Math.max(Math.abs(coord.x1 - coord.x2), Math.abs(coord.y1 - coord.y2));
      }
    },
    
    EUCLIDEAN_NORM{
      @Override
      public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan){
        coords coord = new coords(source, destination, graphWidth);

        return Math.sqrt(Math.pow(coord.x1 - coord.x2, 2) + Math.pow(coord.y1 - coord.y2, 2));
      }
    },
    
    MANHATTAN{
      @Override
      public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan){
        coords coord = new coords(source, destination, graphWidth);

        return Math.abs(coord.x1 - coord.x2) + Math.abs(coord.y1 - coord.y2);        
      }
    },
    
    K_MANHATTAN{
      @Override
      public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan){
        coords coord = new coords(source, destination, graphWidth);

        return kManhattan * (Math.abs(coord.x1 - coord.x2) + Math.abs(coord.y1 - coord.y2));        
      }
    };
    
    private record coords(int x1, int x2, int y1, int y2){
      public coords(int source, int destination, int graphWidth){
        this(
          source % graphWidth,
          source / graphWidth,
          destination % graphWidth,
          destination / graphWidth
        );
      }
    }

    public double computeHeuristic(int source, int destination, int graphWidth, int kManhattan){
      return 0;
    }

  }

  /** Heuristique utilisée pour l'algorithme A*. */
  private final Heuristic heuristic;

  /** Facteur multiplicatif de la distance de Manhattan utilisé par l'heuristique K-Manhattan. */
  private final int kManhattan;

  public AStar(Heuristic heuristic) {
    this(heuristic, 1);
  }

  public AStar(Heuristic heuristic, int kManhattan) {
    this.heuristic = heuristic;
    this.kManhattan = kManhattan;
  }


  @Override
  public Result solve(GridGraph2D grid,
                      PositiveWeightFunction weights,
                      int source,
                      int destination,
                      VertexLabelling<Boolean> processed) {
    throw new UnsupportedOperationException("Not implemented yet");
  }
}