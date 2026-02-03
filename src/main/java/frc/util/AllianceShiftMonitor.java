package frc.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Alliance Shift Monitor for Active HUBs, rumbles controller before shifts and displays shift info
 * on smartdashboard
 */
public class AllianceShiftMonitor {

  // Shift timing (seconds remaining)
  private static final double TRANSITION_END = 120.0; // 2:00
  private static final double SHIFT_1_END = 105.0; // 1:45
  private static final double SHIFT_2_END = 80.0; // 1:20
  private static final double SHIFT_3_END = 55.0; // 0:55
  private static final double SHIFT_4_END = 30.0; // 0:30

  private static final double WARNING_TIME = 5.0; // Rumble 5s before shift
  private static final double RUMBLE_INTENSITY = 0.7;
  private static final double RUMBLE_DURATION = 0.3; // Duration of each rumble
  private static final double RUMBLE_GAP = 0.15; // Gap between rumbles

  private final CommandXboxController driverController;
  private Timer rumbleTimer = new Timer();
  private boolean isRumbling = false;
  private int rumbleCount = 0; // Track which rumble we're on (0-2)

  // Game tracking
  private String gameData = "";
  private boolean gameDataReceived = false;
  private boolean isRedAlliance = false;
  private boolean ourHubActiveInOddShifts = true;
  private int currentShift = 0;
  private boolean hasWarned = false;

  public AllianceShiftMonitor(CommandXboxController driverController) {
    this.driverController = driverController;
    rumbleTimer.start();
  }

  public void periodic() {
    updateAlliance();
    updateGameData();

    if (DriverStation.isTeleopEnabled()) {
      double time = DriverStation.getMatchTime();
      int newShift = getShift(time);

      if (newShift != currentShift) {
        hasWarned = false;
        currentShift = newShift;
      }

      checkShiftWarning(time);
    }

    handleRumble();
    updateDashboard();
  }

  private void updateAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRedAlliance = alliance.get() == DriverStation.Alliance.Red;
    }
  }

  private void updateGameData() {
    String data = DriverStation.getGameSpecificMessage();
    if (!gameDataReceived && data != null && !data.isEmpty()) {
      gameData = data;
      gameDataReceived = true;

      char winner = gameData.charAt(0);
      // Winner's hub is INACTIVE in shifts 1 & 3
      ourHubActiveInOddShifts = isRedAlliance ? (winner == 'B') : (winner == 'R');
    }
  }

  private int getShift(double time) {
    if (time >= TRANSITION_END) return 0;
    if (time >= SHIFT_1_END) return 1;
    if (time >= SHIFT_2_END) return 2;
    if (time >= SHIFT_3_END) return 3;
    if (time >= SHIFT_4_END) return 4;
    return 5; // End game
  }

  private void checkShiftWarning(double time) {
    if (hasWarned) return;

    double[] times = {TRANSITION_END, SHIFT_1_END, SHIFT_2_END, SHIFT_3_END, SHIFT_4_END};

    for (double shiftTime : times) {
      if (time <= shiftTime + WARNING_TIME && time > shiftTime) {
        rumble();
        hasWarned = true;
        break;
      }
    }
  }

  private void rumble() {
    driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
    isRumbling = true;
    rumbleCount = 0; // Start first rumble
    rumbleTimer.reset();
  }

  private void handleRumble() {
    if (!isRumbling) return;

    double elapsed = rumbleTimer.get();

    // Pattern: rumble, gap, rumble, gap, rumble, done
    // Rumble 1: 0.0 - 0.3s
    // Gap 1:    0.3 - 0.45s
    // Rumble 2: 0.45 - 0.75s
    // Gap 2:    0.75 - 0.9s
    // Rumble 3: 0.9 - 1.2s
    // Done:     1.2s+

    if (rumbleCount == 0) {
      // First rumble
      if (elapsed >= RUMBLE_DURATION) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        rumbleCount = 1;
      }
    } else if (rumbleCount == 1) {
      // Gap after first rumble
      if (elapsed >= RUMBLE_DURATION + RUMBLE_GAP) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
        rumbleCount = 2;
      }
    } else if (rumbleCount == 2) {
      // Second rumble
      if (elapsed >= (RUMBLE_DURATION * 2) + RUMBLE_GAP) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        rumbleCount = 3;
      }
    } else if (rumbleCount == 3) {
      // Gap after second rumble
      if (elapsed >= (RUMBLE_DURATION * 2) + (RUMBLE_GAP * 2)) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
        rumbleCount = 4;
      }
    } else if (rumbleCount == 4) {
      // Third rumble
      if (elapsed >= (RUMBLE_DURATION * 3) + (RUMBLE_GAP * 2)) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        isRumbling = false;
        rumbleCount = 0;
      }
    }
  }

  public boolean isHubActive() {
    if (!DriverStation.isTeleopEnabled()) return true;

    int shift = getShift(DriverStation.getMatchTime());
    if (shift == 0 || shift == 5) return true; // Transition & End Game
    if (!gameDataReceived) return true; // No data yet

    boolean isOdd = (shift == 1 || shift == 3);
    return (isOdd && ourHubActiveInOddShifts) || (!isOdd && !ourHubActiveInOddShifts);
  }

  public String getPhase() {
    if (!DriverStation.isTeleopEnabled()) return "DISABLED";

    double time = DriverStation.getMatchTime();
    if (time >= TRANSITION_END) return "TRANSITION";
    if (time >= SHIFT_1_END) return "SHIFT 1";
    if (time >= SHIFT_2_END) return "SHIFT 2";
    if (time >= SHIFT_3_END) return "SHIFT 3";
    if (time >= SHIFT_4_END) return "SHIFT 4";
    return "END GAME";
  }

  private void updateDashboard() {
    SmartDashboard.putString("Shift/Phase", getPhase());
    SmartDashboard.putBoolean("Shift/Hub Active", isHubActive());
    SmartDashboard.putString("Shift/Status", isHubActive() ? "✓ SCORE!" : "○ COLLECT");
    SmartDashboard.putString("Shift/Alliance", isRedAlliance ? "RED" : "BLUE");

    if (DriverStation.isTeleopEnabled()) {
      double time = DriverStation.getMatchTime();
      double next = getNextShiftTime(currentShift);
      if (next > 0) {
        SmartDashboard.putNumber("Shift/Next In", time - next);
      }
    }
  }

  private double getNextShiftTime(int shift) {
    switch (shift) {
      case 0:
        return TRANSITION_END;
      case 1:
        return SHIFT_1_END;
      case 2:
        return SHIFT_2_END;
      case 3:
        return SHIFT_3_END;
      case 4:
        return SHIFT_4_END;
      default:
        return -1;
    }
  }

  public void reset() {
    gameData = "";
    gameDataReceived = false;
    hasWarned = false;
    currentShift = 0;
    rumbleCount = 0;
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    isRumbling = false;
  }
}
