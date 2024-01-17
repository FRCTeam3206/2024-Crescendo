// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Whether the button for going to pose is pressed
  private boolean goToPosPressed = false;
  // If set to true and robot is going to pose, stops
  private boolean interruptGoToPos = false;

  // setGoToPos, isGoingToPos

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  SendableChooser<Boolean> m_resetGyroChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_resetGyroChooser.setDefaultOption("Don't reset gyro", false);
    m_resetGyroChooser.addOption("Reset gyro", true);
    SmartDashboard.putData("Reset Gyro", m_resetGyroChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        m_robotDrive.driveCommand(
            () ->
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            () ->
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            () ->
                -MathUtil.applyDeadband(
                    m_driverController.getRawAxis(2), OIConstants.kDriveDeadband),
            true,
            true));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Constants.Buttons.kSetXButton)
        .whileTrue(m_robotDrive.setXCommand()); // Button.kR1.value

    Trigger m_resetGyro = new Trigger(() -> m_resetGyroChooser.getSelected());
    m_resetGyro.onTrue(
        new InstantCommand(
            () -> {
              m_robotDrive.zeroHeading();
            },
            m_robotDrive));

    // This uses when the button is pressed to set goToPosPressed
    JoystickButton m_goToPosPressed =
        new JoystickButton(m_driverController, Constants.Buttons.kGoToButton);
    m_goToPosPressed.onTrue(
        new InstantCommand(
            () -> {
              goToPosPressed = true;
            }));
    m_goToPosPressed.onFalse(
        new InstantCommand(
            () -> {
              goToPosPressed = false;
            }));

    SmartDashboard.putNumber("xPoseWhenSet", 0);
    SmartDashboard.putNumber("yPoseWhenSet", 0);
    new Trigger(() -> goToPosPressed)
        .onTrue(
            goToPosStop(
                new Pose2d(
                    SmartDashboard.getNumber("xPoseWhenSet", 0),
                    SmartDashboard.getNumber("yPoseWhenSet", 0),
                    new Rotation2d(0)),
                10));

    JoystickButton m_interruptGoToPos =
        new JoystickButton(m_driverController, Constants.Buttons.kInterruptGoTo);
    m_interruptGoToPos.onTrue(
        new InstantCommand(
            () -> {
              interruptGoToPos = true;
            }));
    // Trigger m_goToPosToggled = new Trigger(() -> {return goToPosPressed;});
    // m_goToPosToggled.onTrue(new InstantCommand(() -> {goToPosToggled = true;}));

    // Trigger m_setGoToPosition = new Trigger(() -> goToPosToggled);

    // m_setGoToPos.onTrue(goToPosStop(1, 3).andThen(new InstantCommand(() -> {setGoToPos =
    // false;})));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 1, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  /**
   * Does not currently go to the rotation
   *
   * @return Command to go from the robot's current position to the given Pose2d, then stop
   * @param pose The goal position
   * @param centimetersOff The number of centimeters the robot should be within to end the path
   */
  public Command goToPosStop(Pose2d pose, double centimetersOff) {
    interruptGoToPos = false;
    // Create config for trajectory
    TrajectoryConfig config2 =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory toPosTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(
                m_robotDrive.getPose().getX(),
                m_robotDrive.getPose().getY(),
                m_robotDrive.getPose().getRotation()),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            pose,
            config2);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand toPosCommand =
        new SwerveControllerCommand(
            toPosTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Run path following command, then stop at the end.
    return new FunctionalCommand(
        null,
        (Runnable)
            toPosCommand.andThen(
                () -> {
                  m_robotDrive.drive(0, 0, 0, false, false);
                }),
        null,
        () -> {
          return Math.abs(m_robotDrive.getPose().getX() - pose.getX()) < centimetersOff / 100
              && Math.abs(m_robotDrive.getPose().getY() - pose.getY()) < centimetersOff / 100
              && interruptGoToPos;
        },
        m_robotDrive);
  }

  public Command goToPoseWithFeedback(Pose2d pose, double centimetersOff) {
    double startXDistFromGoal = pose.getX() - m_robotDrive.getPose().getX();
    double startYDistFromGoal = pose.getY() - m_robotDrive.getPose().getY();
    DoubleSupplier getXSpeed =
        () -> {
          return ((pose.getX() - m_robotDrive.getPose().getX()) / startXDistFromGoal);
        };
    DoubleSupplier getYSpeed =
        () -> {
          return ((pose.getY() - m_robotDrive.getPose().getY()) / startYDistFromGoal);
        };
    return new FunctionalCommand(
        null,
        (Runnable) m_robotDrive.driveCommand(getXSpeed, getYSpeed, () -> 0, true, true),
        null,
        () -> {
          return Math.abs(m_robotDrive.getPose().getX() - pose.getX()) < centimetersOff / 100
              && Math.abs(m_robotDrive.getPose().getY() - pose.getY()) < centimetersOff / 100
              && interruptGoToPos;
        },
        m_robotDrive);
  }
}
