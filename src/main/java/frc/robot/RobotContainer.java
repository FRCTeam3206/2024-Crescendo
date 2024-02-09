// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shootake;
import java.util.List;
import monologue.Annotations.Log;
import monologue.Logged;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shootake shootake = new Shootake();
  private final Arm arm = new Arm();
  @Log.NT private final String currentBranch = BuildConstants.GIT_BRANCH;

  // The driver's controller
  @Log.NT
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

  CommandXboxController xbox = new CommandXboxController(1);
  SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    autons();

    // Configure default commands
    SmartDashboard.putBoolean("Field Relative", true);

    m_robotDrive.setDefaultCommand(
        // Uses a joystick.
        // x and y motion is controlled by the x and y axis of the stick.
        // turning is controlled by rotating (twisting) the stick
        m_robotDrive.driveCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
            () ->
                -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
            () -> {
              return SmartDashboard.getBoolean("Field Relative", true);
            },
            true));
    shootake.setDefaultCommand(shootake.idleCommand());
    arm.setDefaultCommand(
        new RunCommand(
            () -> {
              arm.setVoltage(MathUtil.applyDeadband(xbox.getLeftY() * 2, 0.00));
            },
            arm));
  }

  // new RunCommand(
  //             () -> {
  //               shootake.setSpeed(MathUtil.applyDeadband(xbox.getRightY(), kXboxDeadband));
  //               shootake.setRetained(xbox.getHID().getAButton());
  //             },
  //             shootake)

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_driverController.button(2).whileTrue(m_robotDrive.setXCommand());
    xbox.a().whileTrue(shootake.intakeCommand());
    xbox.b().onTrue(shootake.shootCommand());
    xbox.y().whileTrue(arm.holdAngle(0));

    SmartDashboard.putData("Reset Gyro", m_robotDrive.zeroHeadingCommand());

    SmartDashboard.putNumber("X to Reset", 0);
    SmartDashboard.putNumber("Y to Reset", 0);
    SmartDashboard.putData(
        "Reset Pose",
        new InstantCommand(
            () -> {
              m_robotDrive.resetOdometry(
                  new Pose2d(
                      SmartDashboard.getNumber("X to Reset", 0),
                      SmartDashboard.getNumber("Y to Reset", 0),
                      m_robotDrive.getPose().getRotation()));
            }));
  }

  public void autons() {
    autonChooser.setDefaultOption(
        "Nothing", m_robotDrive.driveCommand(() -> 0, () -> 0, () -> 0, () -> true, true));

    autonChooser.addOption(
        "S Path",
        generateAutonomousCommand(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Math.PI))));

    autonChooser.addOption(
        "Forward 2 Meters",
        generateAutonomousCommand(
            new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(2, 0, new Rotation2d(0))));

    autonChooser.addOption(
        "Figure 8",
        generateAutonomousCommand(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(5.5, 1),
                new Translation2d(8.3, 4),
                new Translation2d(11, 7),
                new Translation2d(15.7, 4),
                new Translation2d(11, 1),
                new Translation2d(8.3, 4),
                new Translation2d(5.5, 7),
                new Translation2d(.7, 4)),
            new Pose2d(1, 1, new Rotation2d(0))));

    Pose2d start = new Pose2d(1.9, 7.8 - Units.feetToMeters(6), new Rotation2d(0));
    Pose2d note = new Pose2d(2.3, 5.55, new Rotation2d(0));
    Pose2d amp = new Pose2d(1.9, 7.8, new Rotation2d(Math.PI));
    autonChooser.addOption(
        "Score note in amp",
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  m_robotDrive.resetOdometry(start);
                }),
            generateAutonomousCommand(start, List.of(), note),
            // pickUpNote(),
            generateAutonomousCommand(note, List.of(), amp)
            // scoreToAmp()
            ));

    SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    if (autonChooser.getSelected() == null) {
      return m_robotDrive.driveCommand(() -> 0, () -> 0, () -> 0, () -> true, true);
    }
    return autonChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command generateAutonomousCommand(
      Pose2d startPose, List<Translation2d> waypoints, Pose2d endPose) {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond * 0.5,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(m_robotDrive::stopCommand);
  }
}
