package ch.heig.gre.groupN;

import ch.heig.gre.maze.MazeBuilder;
import ch.heig.gre.maze.MazeGenerator;
import ch.heig.gre.maze.Progression;

import java.util.List;
import java.util.Random;
import java.util.Stack;

public final class DfsGenerator implements MazeGenerator {

  Random random = new Random();

  @Override
  public void generate(MazeBuilder builder, int from) {
    Stack<Integer> parents = new Stack<>();
    Integer cursor = from;
    Integer next;
    List<Integer> neighbors;

    while(true){
      builder.progressions().setLabel(cursor, Progression.PROCESSING);

      neighbors = builder.topology().neighbors(cursor);
      neighbors.removeIf(n -> builder.progressions().getLabel(n) != Progression.PENDING);

      if(neighbors.size() > 0){
        parents.push(cursor);
        next = neighbors.get(random.nextInt(neighbors.size()));
        builder.removeWall(cursor, next);
        cursor = next;
      }else{
        builder.progressions().setLabel(cursor, Progression.PROCESSED);
        cursor = parents.pop();
      }
    }
  }

  @Override
  public boolean requireWalls() {
    return true;
  }

}
