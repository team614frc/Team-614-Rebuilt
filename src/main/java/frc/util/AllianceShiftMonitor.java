package frc.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AllianceShiftMonitor {

  private enum Phase {
    TRANSITION(130), // 2:10 remaining (after 20s AUTO)
    SHIFT_1(105), // 1:45 remaining
    SHIFT_2(80), // 1:20 remaining
    SHIFT_3(55), // 0:55 remaining
    SHIFT_4(30), // 0:30 remaining
    END_GAME(0); // 0:00 remaining

    private final double seconds;

    private Phase(double seconds) {
      this.seconds = seconds;
    }

    private static Phase fromMatchTime(double seconds) {
      for (Phase phase : values()) {
        if (seconds >= phase.seconds) return phase;
      }
      return END_GAME;
    }
  }

  private static final Time WARNING = Seconds.of(5);
  private static final Time SHIFT_QUIET = Seconds.of(20);
  private static final Time RUMBLE_PULSE = Seconds.of(0.3);
  private static final Time RUMBLE_GAP = Seconds.of(0.15);

  private static final double RUMBLE_INTENSITY = 0.7;

  private final CommandXboxController driverController;

  private boolean hasGameData = false;
  private boolean isRedAlliance = false;
  private boolean isOurHubActiveInOddShifts = true;
  private boolean isScheduled = false;
  private Command scheduledCommand = null;

  public AllianceShiftMonitor(CommandXboxController driverController) {
    this.driverController = driverController;
  }

  public void periodic() {
    DriverStation.getAlliance().ifPresent(a -> isRedAlliance = a == DriverStation.Alliance.Red);

    if (!hasGameData) {
      String data = DriverStation.getGameSpecificMessage();
      if (data != null && !data.isEmpty()) {
        char winner = data.charAt(0);
        isOurHubActiveInOddShifts = isRedAlliance ? (winner == 'B') : (winner == 'R');
        hasGameData = true;
      }
    }

    if (DriverStation.isTeleopEnabled() && hasGameData && !isScheduled) {
      scheduledCommand = isOurHubActiveInOddShifts ? buildLoserCommand() : buildWinnerCommand();
      CommandScheduler.getInstance().schedule(scheduledCommand);
      isScheduled = true;
    }

    updateDashboard();
  }

  /** Loser: odd shifts (1, 3) are ours. */
  private Command buildLoserCommand() {
    return Commands.sequence(
        // TRANSITION: 10s total, rumble3x at 5s mark
        Commands.deadline(wait(Seconds.of(10)), Commands.sequence(wait(WARNING), rumble3x())),
        // SHIFT 1 (ours): 20s quiet + 5s rumble = 25s total
        wait((SHIFT_QUIET)),
        rumble((WARNING)),
        // SHIFT 2 (opponent): 25s total, rumble3x at 20s mark
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x())),
        // SHIFT 3 (ours): 20s quiet + 5s rumble = 25s total
        wait(SHIFT_QUIET),
        rumble(WARNING),
        // SHIFT 4 (opponent): 25s total, rumble3x at 20s mark
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x()))
        // Total: 10 + 25 + 25 + 25 + 25 = 110s (+ 20s AUTO = 130s)
        );
  }

  /** Winner: even shifts (2, 4) are ours. */
  private Command buildWinnerCommand() {
    return Commands.sequence(
        // TRANSITION: 10s total, rumble at 5s mark (transition ending soon)
        Commands.deadline(wait(Seconds.of(10)), Commands.sequence(wait(WARNING), rumble(WARNING))),
        // SHIFT 1 (opponent): 25s total, rumble3x at 20s mark (for our shift 2)
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x())),
        // SHIFT 2 (ours): 20s quiet + 5s rumble = 25s total
        wait(SHIFT_QUIET),
        rumble(WARNING),
        // SHIFT 3 (opponent): 25s total, rumble3x at 20s mark (for our shift 4)
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x())),
        // SHIFT 4 (ours): 20s quiet + 5s rumble = 25s total
        wait(SHIFT_QUIET),
        rumble(WARNING)
        // Total: 10 + 25 + 25 + 25 + 25 = 110s (+ 20s AUTO = 130s)
        );
  }

  private Command wait(Time duration) {
    return Commands.waitSeconds(duration.in(Seconds));
  }

  private Command rumble(Time duration) {
    return new RunCommand(
            () -> driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY))
        .finallyDo(i -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        .withTimeout(duration.in(Seconds));
  }

  private Command rumble3x() {
    return Commands.sequence(
        rumble(RUMBLE_PULSE),
        wait(RUMBLE_GAP),
        rumble(RUMBLE_PULSE),
        wait(RUMBLE_GAP),
        rumble(RUMBLE_PULSE));
  }

  private void updateDashboard() {
    String phase = getPhase();

    SmartDashboard.putString("Shift/Phase", phase);
    SmartDashboard.putString("Shift/Alliance", isRedAlliance ? "RED" : "BLUE");
    SmartDashboard.putBoolean("Shift/Game Data Received", hasGameData);
    SmartDashboard.putBoolean("Shift/Scheduled", isScheduled);

    if (!"DISABLED".equals(phase)) {
      double seconds = DriverStation.getMatchTime();
      Phase p = Phase.fromMatchTime(seconds);
      SmartDashboard.putNumber("Shift/Time Left", Math.max(0, seconds - p.seconds));
    }
  }

  public String getPhase() {
    if (!DriverStation.isTeleopEnabled()) return "DISABLED";
    return Phase.fromMatchTime(DriverStation.getMatchTime()).name();
  }

  public void reset() {
    hasGameData = false;
    isScheduled = false;
    if (scheduledCommand != null) {
      scheduledCommand.cancel();
      scheduledCommand = null;
    }
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }
}
