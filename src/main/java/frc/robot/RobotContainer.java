package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterVisualizer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.util.AllianceShiftMonitor;
import frc.util.FuelSim;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Intake intake = new Intake();
  private final Floor floor = new Floor();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Hanger hanger = new Hanger();
  private final VisionSubsystem vision = new VisionSubsystem(swerve);

  // Shooter visualizer for simulation (only used in sim)
  private final ShooterVisualizer shooterVisualizer;

  private final AllianceShiftMonitor shiftMonitor;

  // The driver's controller
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController codriverXbox =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              swerve.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> -driverXbox.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              swerve.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);

  private final SubsystemCommands subsystemCommands;

  private void configureFuelSim() {
    System.out.println("=== CONFIGURING FUELSIM ===");

    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();

    instance.registerRobot(
        0.686, // length: 27"
        0.648, // width: 25.5"
        0.406, // height: ~16" (bumpers + frame average)
        () -> swerve.getPose(),
        () -> swerve.getRobotVelocity());
    // FRONT INTAKE - extends 10" beyond frame perimeter
    instance.registerIntake(
        0.547,
        0.647, // X range: front of robot (0.597m ± 0.05m for depth)
        -0.324,
        0.324, // Y range: full robot width (±0.324m = 25.5" / 2)
        () -> {
          // Only intake if command is active AND we have space
          boolean commandActive = intake.getCurrentCommand() != null;
          boolean hasSpace = shooterVisualizer != null && shooterVisualizer.simCanIntake();
          boolean shouldIntake = commandActive && hasSpace;

          if (commandActive && !hasSpace) {
            System.out.println("Intake active but FULL - not picking up fuel");
          }

          return shouldIntake;
        },
        () -> {
          System.out.println("*** FUEL PICKED UP! ***");
          if (RobotBase.isSimulation() && shooterVisualizer != null) {
            shooterVisualizer.simIntake();
            System.out.println("Total fuel: " + shooterVisualizer.getFuelCount());
          }
        });

    instance.start();

    // Test buttons for FuelSim
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                  if (shooterVisualizer != null) shooterVisualizer.resetFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));

    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  if (shooterVisualizer != null) {
                    shooterVisualizer.launchFuel();
                  }
                })
            .withName("Test Launch Fuel")
            .ignoringDisable(true));

    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  if (shooterVisualizer != null) {
                    shooterVisualizer.setFuelCount(3);
                  }
                })
            .withName("Give 3 Fuel")
            .ignoringDisable(true));

    System.out.println("=== FUELSIM COMPLETE ===");
  }

  public RobotContainer() {
    // Initialize shooter visualizer
    if (RobotBase.isSimulation()) {
      shooterVisualizer =
          new ShooterVisualizer(
              () -> swerve.getPose(),
              () -> hood.getAngle(),
              () -> shooter.getExitVelocity() // already matches 3-param constructor
              );
      configureFuelSim();
    } else {
      shooterVisualizer = null;
    }

    // Initialize subsystemCommands with the shooterVisualizer
    subsystemCommands =
        new SubsystemCommands(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            vision,
            shooterVisualizer, // Now this is initialized!
            () -> driverXbox.getLeftY(),
            () -> driverXbox.getLeftX());

    shiftMonitor = new AllianceShiftMonitor(driverXbox);
    DriverStation.silenceJoystickConnectionWarning(true);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    RobotModeTriggers.autonomous()
        .or(RobotModeTriggers.teleop())
        .onTrue(intake.homingCommand())
        .onTrue(hanger.homingCommand());

    driverXbox.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
    driverXbox.rightBumper().whileTrue(subsystemCommands.shootManually());
    driverXbox.leftTrigger().whileTrue(intake.intakeCommand());
    driverXbox.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

    driverXbox.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
    driverXbox.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void periodic() {
    shooterVisualizer.periodic();
    shiftMonitor.periodic();
  }

  /** Call from Robot.autonomousInit() */
  public void resetShiftMonitor() {
    shiftMonitor.reset();
  }
}
