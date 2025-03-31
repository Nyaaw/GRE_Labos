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
      public int computeHeuristic(int source, int destination, int k){
        return 0;        
      }
    },
    
    EUCLIDEAN_NORM{
      @Override
      public int computeHeuristic(int source, int destination, int k){
        return 0;          
      }
    },
    
    MANHATTAN{
      @Override
      public int computeHeuristic(int source, int destination, int k){
        return 0;        
      }
    },
    
    K_MANHATTAN{
      @Override
      public int computeHeuristic(int source, int destination, int k){
        return 0;        
      }
    };

    public int computeHeuristic(int source, int destination, int k){
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