package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class Landmarks {
  public static Translation2d hubPosition() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new Translation2d(Inches.of(181.156), Inches.of(158.32));
    }
    return new Translation2d(Inches.of(468.56), Inches.of(158.32));
  }
}
