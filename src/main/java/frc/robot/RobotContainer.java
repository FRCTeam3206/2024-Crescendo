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
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.ArmSubsystem;
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
  private final ArmSubsystem arm = new ArmSubsystem();
  private final Climber climber = new Climber();
  @Log.Once private final String currentBranch = BuildConstants.GIT_BRANCH;

  // The driver's controller
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

  CommandXboxController xbox = new CommandXboxController(1);
  SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public static enum AllianceColor {
    RED,
    BLUE,
    UNKNOWN;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    // Configure the button bindings
    configureButtonBindings();

    autons();

    // TODO (if you haven't done the parts in Climber yet, do that first).
    // Set the default command for "climber" with the method you defined in the class.
    // For the sake of this exercise, you can just set the climber speeds to a value that you choose.
    // Hint: Look at how the default commands are set for m_robotDrive and shootake.

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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  public void autons() {
    autonChooser.setDefaultOption("Nothing", m_robotDrive.stopCommand());
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
}
