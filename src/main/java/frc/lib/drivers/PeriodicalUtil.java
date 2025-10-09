package frc.lib.drivers;

import java.util.ArrayList;
import java.util.List;

public class PeriodicalUtil {
  private static List<Periodical> periodicals = new ArrayList<>();

  public static void registerPeriodic(Periodical periodic) {
    periodicals.add(periodic);
  }

  public static void runPeriodic() {
    periodicals.forEach((periodical) -> periodical.periodic());
  }
}

interface Periodical {
  public void periodic();
}
