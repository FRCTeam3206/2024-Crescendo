// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AllianceNoteLocation;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RelativeTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lights;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  Lights lights = new Lights();
  final Shootake shootake = new Shootake();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();
  @Log private final String currentBranch = BuildConstants.GIT_BRANCH;

  // The driver's controller
  @Log CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

  CommandXboxController xbox = new CommandXboxController(1);
  SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public static enum AllianceColor {
    RED,
    BLUE,
    UNKNOWN;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    autons();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // Uses a joystick.
        // x and y motion is controlled by the x and y axis of the stick.
        // turning is controlled by rotating (twisting) the stick
        m_robotDrive.driveCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
            () ->
                -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
            () -> RelativeTo.DRIVER_RELATIVE,
            true));
    shootake.setDefaultCommand(shootake.idleCommand());
    climber.setDefaultCommand(
        new RunCommand(
            () -> {
              climber.setSpeed(MathUtil.applyDeadband(xbox.getRightY(), 0.1));
            },
            climber));
    lights.setDefaultCommand(
        new RunCommand(
            () -> {
              if (shootake.hasNote()) {
                if (m_robotDrive.isSpeakerAligned()) {
                  lights.setLightColor(0, 255, 0);
                } else {
                  lights.setLightColor(255, 80, 0);
                }
              } else {
                lights.setLightColor(0, 0, 255);
              }
            },
            lights));
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
    // m_driverController.button(2).whileTrue(m_robotDrive.pathCommandToPose(new Pose2d(13.349,
    // 5.326,new Rotation2d(Math.PI))));
    m_driverController
        .button(2)
        .whileTrue(
            // m_robotDrive.autoDriveToSpeakerShoot()
            speakerShoot());
    m_driverController.button(5).whileTrue(m_robotDrive.scoreToAmpCommand());
    xbox.povUp().onTrue(arm.intakePosition());
    xbox.povDown().onTrue(arm.shootPosition());
    xbox.povRight().onTrue(arm.ampPosition());
    xbox.povLeft().onTrue(arm.subwooferPosition());
    xbox.a().whileTrue(shootake.intakeCommand());
    xbox.b().onTrue(shootake.speakerShootCommand());
    xbox.y().whileTrue(shootake.ampCommand());
    xbox.x().whileTrue(shootake.outakeCommand());
    xbox.start().whileTrue(shootake.slowIntakeCommand());

    SmartDashboard.putData("Reset Gyro", m_robotDrive.zeroHeadingCommand());
    SmartDashboard.putData("Reset Climber", climber.zero());
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

  // public Command pickUpNoteCommand(NoteLocation noteLocation) {
  //   return new ParallelRaceGroup(m_robotDrive.pickUpNotePoseCommand(noteLocation), new
  // ConditionalCommand(getAutonomousCommand(), getAutonomousCommand(), () -> )));
  // }

  public Command pickUpNoteCommand() {
    return new SequentialCommandGroup(
        m_robotDrive.stopCommand(),
        new ParallelRaceGroup(arm.intakeCommandStop(), shootake.intakeCommand()),
        new ParallelCommandGroup(
                m_robotDrive.driveCommand(
                    () -> 0.15, () -> 0.0, () -> 0.0, () -> RelativeTo.ROBOT_RELATIVE, true),
                arm.intakePosition(),
                shootake.intakeCommand())
            .until(() -> shootake.hasNote())
            .withTimeout(2.0),
        m_robotDrive.stopCommand(),
        shootake.intakeCommand().withTimeout(.25),
        shootake.retainCommand(),
        shootake.stopCommand());
  }

  public Command pickUpNoteCommand(AllianceNoteLocation noteLocation) {
    return pickUpNoteCommand(noteLocation.getPickUpPose());
  }

  public Command pickUpNoteCommand(Pose2d pickupPose) {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            m_robotDrive.driveToPoseCommand(
                pickupPose, AutoAlignConstants.kAtNotePickupGoalTolerance),
            arm.intakePosition()),
        pickUpNoteCommand(),
        m_robotDrive.stopCommand());
  }

  public Command speakerShoot() {
    return new SequentialCommandGroup(
        m_robotDrive.stopCommand(),
        new ParallelCommandGroup(
            arm.speakerCommandStop(),
            new RunCommand(() -> shootake.setRetained(true), shootake).withTimeout(.5),
            m_robotDrive.autoDriveToSpeakerShoot()),
        m_robotDrive.stopCommand(),
        shootake.speakerShootCommand());
  }

  public Command ampShoot() {
    return new SequentialCommandGroup(
        m_robotDrive.stopCommand(),
        new ParallelCommandGroup(arm.ampCommandStop(), m_robotDrive.scoreToAmpCommand()),
        m_robotDrive.stopCommand(),
        new ParallelCommandGroup(
            m_robotDrive.stopCommand(), arm.ampPosition(), shootake.ampCommand()));
  }

  public Command bottomToSpeaker() {
    return new SequentialCommandGroup(
        pickUpNoteCommand(AllianceNoteLocation.BOTTOM), speakerShoot());
  }

  public Command midToSpeaker() {
    return new SequentialCommandGroup(
        pickUpNoteCommand(AllianceNoteLocation.CENTER), speakerShoot());
  }

  public Command topToSpeaker() {
    return new SequentialCommandGroup(pickUpNoteCommand(AllianceNoteLocation.TOP), speakerShoot());
  }

  public Command topToAmp() {
    return new SequentialCommandGroup(pickUpNoteCommand(AllianceNoteLocation.TOP), ampShoot());
  }

  public Command topToAmpWallSide() {
    return new SequentialCommandGroup(
        pickUpNoteCommand(
            AllianceNoteLocation.TOP
                .getPose()
                .transformBy(
                    new Transform2d(-AutoAlignConstants.kPickUpNoteDist, 0, new Rotation2d()))),
        ampShoot());
  }

  public void autons() {
    autonChooser.setDefaultOption("Nothing", m_robotDrive.stopCommand());

    autonChooser.addOption(
        "1 Note",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                    m_robotDrive.driveCommand(
                        () -> .25, () -> 0, () -> 0, () -> RelativeTo.DRIVER_RELATIVE, false),
                    new RunCommand(() -> shootake.setRetained(true), shootake))
                .withTimeout(1.5),
            m_robotDrive.stopCommand(),
            speakerShoot()));
    autonChooser.addOption(
        "Subwoofer",
        new SequentialCommandGroup(
            arm.subwooferPosition().withTimeout(2),
            shootake.speakerShootCommand(),
            m_robotDrive
                .driveCommand(() -> .25, () -> 0, () -> 0, () -> RelativeTo.DRIVER_RELATIVE, false)
                .withTimeout(1.5),
            new RunCommand(() -> shootake.setRetained(true), shootake),
            m_robotDrive.stopCommand()));
    autonChooser.addOption(
        "2 Note (Middle Shoot)",
        new SequentialCommandGroup(speakerShoot(), shootake.stopCommand(), midToSpeaker()));
    autonChooser.addOption(
        "2 Note (Amp-Side Shoot)",
        new SequentialCommandGroup(speakerShoot(), shootake.stopCommand(), topToSpeaker()));
    autonChooser.addOption(
        "2 Note (Stage-Side Shoot)",
        new SequentialCommandGroup(speakerShoot(), shootake.stopCommand(), bottomToSpeaker()));
    autonChooser.addOption(
        "2 Note (Amp Score)",
        new SequentialCommandGroup(speakerShoot(), shootake.stopCommand(), topToAmpWallSide()));
    autonChooser.addOption(
        "3 Note (Amp Side All Speaker)",
        new SequentialCommandGroup(
            speakerShoot(), shootake.stopCommand(), midToSpeaker(), topToSpeaker()));
    autonChooser.addOption(
        "3 Note (Stage Side)",
        new SequentialCommandGroup(
            speakerShoot(), shootake.stopCommand(), midToSpeaker(), bottomToSpeaker()));
    autonChooser.addOption(
        "3 Note (Amp Side One Amp)",
        new SequentialCommandGroup(
            speakerShoot(), shootake.stopCommand(), midToSpeaker(), topToAmp()));
    autonChooser.addOption(
        "4 Note",
        new SequentialCommandGroup(
            speakerShoot(), shootake.stopCommand(), bottomToSpeaker(), midToSpeaker(), topToAmp()));
    SmartDashboard.putData(autonChooser);
  }

  public Command getAutonomousCommand() {
    if (autonChooser.getSelected() == null) {
      return m_robotDrive.stopCommand();
    }
    return new ConditionalCommand(
        m_robotDrive.stopCommand(),
        autonChooser.getSelected(),
        () -> m_robotDrive.getPose().getX() < .1 && m_robotDrive.getPose().getY() < .1);
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

  // public void setAdjustmentForGyro(double adjustment) {
  //   m_robotDrive.setGyroAdjustment(adjustment);
  // }
}
