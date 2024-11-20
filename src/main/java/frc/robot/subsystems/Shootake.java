package frc.robot.subsystems;

import static frc.robot.Constants.ShootakeConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootakeConstants;
import java.util.function.BooleanSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

// For this exercise, fix problems and add pieces of code where there are comments saying to do so.
//

public class Shootake extends SubsystemBase implements Logged {
  CANSparkMax topRoller = new CANSparkMax(ShootakeConstants.kTopCANID, MotorType.kBrushless); // TODO: Give the constructor the necessary information
  CANSparkMax bottomRoller = new CANSparkMax(ShootakeConstants.kBottomCANID, MotorType.kBrushless); // TODO: Give the constructor the necessary information
  // A constructor is a method that allows you to create an instance of an object.
  // For more information about any of the following, you can visit the following websites:
  // Method: https://www.w3schools.com/java/java_methods.asp
  // Object: https://www.w3schools.com/java/java_classes.asp
  // Constructor: https://www.w3schools.com/java/java_constructors.asp
  // Hint: hovering over "CANSparkMax()" will give you more information about it
  // Hint: there is some important information you will need in ShootakeConstants
  Servo finger = new Servo(kFingerPort);
  DigitalInput hasNoteSensor = new DigitalInput(ShootakeConstants.kNoteSensorChannel);
  Debouncer shootDebounce = new Debouncer(.125);

  public Shootake() {
    topRoller.setSmartCurrentLimit(23);
    bottomRoller.setSmartCurrentLimit(23);
  }

  @Log
  private double getAverageSpeed() {
    return -(topRoller.getEncoder().getVelocity() + bottomRoller.getEncoder().getVelocity()) / 2;
  }

  @Log
  private double getTopRollerSpeed() {
    return -topRoller.getEncoder().getVelocity();
  }

  @Log
  private double getBottomRollerSpeed() {
    return -bottomRoller.getEncoder().getVelocity();
  }

  public void setSpeed(double speed) {
    // TODO: Set the speeds of topRoller and bottomRoller to the given speed.
    // Hint: You will need to use the "set" method in the CANSparkMax objects
    topRoller.set(speed);
    bottomRoller.set(speed);
  }

  public void setRetained(boolean retained) {
    SmartDashboard.putNumber("Servo Retainer", retained ? 1 : 0);
    // TODO: Set the finger to the retained value (found in ShootakeConstants) if the "retained"
    // true/false (boolean) variable is true; otherwise, set it to the
    // not-retained value (also found in ShootakeConstants).
    // Hint: you will need to make an if-else statement
    // For more information about if-else statements, you can look at https://www.w3schools.com/java/java_conditions.asp
    // Hint: you will need to use the "set" method of "finger"
    if (retained){
      finger.set(1);   
     } else {
      finger.set(0);
    }
  }

  public boolean hasNote() {
    return !hasNoteSensor.get();
  }

  public Command idleCommand() {
    return this.run(
        () -> {
          setSpeed(0.0);
          setRetained(true);
        });
  }

  public Command stopCommand() {
    return this.runOnce(() -> this.setSpeed(0));
  }

  // TODO: Make a method in the space below that will return a Command to intake a note.
  // Hint: look at the idleCommand to help you.
  //Think about what speed you will want to set it to and whether it should be retained.
}
