package ch.heig.gre.groupN;

import ch.heig.gre.maze.MazeBuilder;
import ch.heig.gre.maze.MazeGenerator;
import ch.heig.gre.maze.Progression;

import java.util.List;
import java.util.Random;
import java.util.Stack;

/**
 * DfsGenerator implements the DFS algorithm to explore a connex graph and make
 * this exploration the paths of a maze
 *
 * @author Ripoll Pierric - Sottile Alan
 */
public final class DfsGenerator implements MazeGenerator {

  Random random = new Random();

  @Override
  public void generate(MazeBuilder builder, int from) {

    // Contains the path between the root and the current cell
    Stack<Integer> parents = new Stack<>();

    // cell we are currently working on
    Integer current = from;
    Integer next;

    // lists of neighbouring used to choose a random one
    List<Integer> neighbors;

    while (true) {

      // sets the current cell as processing
      builder.progressions().setLabel(current, Progression.PROCESSING);

      // gets the neighbors cells and keeps only those that are pending
      neighbors = builder.topology().neighbors(current);
      neighbors.removeIf(n -> builder.progressions().getLabel(n) != Progression.PENDING);

      if (!neighbors.isEmpty()) {
        parents.push(current);

        // chooses a random neighbor and removes the wall between it and current cell
        next = neighbors.get(random.nextInt(neighbors.size()));
        builder.removeWall(current, next);

        current = next;
      } else {

        // no neighbors left to work on, so we set this cell as processed and
        // go back up the parents stack.
        builder.progressions().setLabel(current, Progression.PROCESSED);
        if (parents.isEmpty()) {
          break;
        }
        current = parents.pop();
      }
    }
  }

  @Override
  public boolean requireWalls() {
    return true;
  }

}
