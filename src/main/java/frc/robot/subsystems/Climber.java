package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import monologue.Logged;

// TODO: our subsystem class needs to be based on another class called SubsystemBase.
// Hint: In order to make a class based on another class, you use the word "extends". The new class is
// a subclass of the class that it's based on. For example, you could do "Blueberry extends Fruit"
// (Blueberry is then a subclass of Fruit) or "Car extends Vehicle" (Car is then a subclass of Vehicle).
public class Climber implements Logged {
  // TODO: add missing arguments (hint: it's the same type of object as the shootake motors are)
  CANSparkMax leftClimber =
      new CANSparkMax();
  CANSparkMax rightClimber =
      new CANSparkMax();

  public Climber() {
    leftClimber.setSmartCurrentLimit(30);
    rightClimber.setSmartCurrentLimit(30);
  }

  public void setSpeedOverride(double speed) {
    leftClimber.set(speed);
    rightClimber.set(-speed);
  }

  public void setSpeed(double speed) {
    if ((leftClimber.getEncoder().getPosition() < ClimberConstants.climbMax && speed > 0)
        || speed < 0) {
      leftClimber.set(speed);
    } else leftClimber.set(0);
    if ((rightClimber.getEncoder().getPosition() * -1 < ClimberConstants.climbMax && speed > 0)
        || speed < 0) {
      rightClimber.set(-speed);
    } else rightClimber.set(0);
  }

  public Command zero() {
    // TODO: give the methods the correct argument for the positions to be zeroed
    return this.runOnce(
        () -> {
          rightClimber.getEncoder().setPosition();
          leftClimber.getEncoder().setPosition();
        });
  }

  public void periodic() {
    // TODO: add a descriptor (key) in place of the empty String to identify the information being logged
    this.log("", rightClimber.getEncoder().getPosition() * -1);
    this.log("", leftClimber.getEncoder().getPosition());
  }

  // TODO: Make a method that will return a Command to set the speed to a given value.
  // Hints:
  // - You will need to use this.run() or this.runOnce()
  // - Look at the end of Shootake for help with the syntax (or ask questions)
  // - Make sure that the speed is a parameter and that you're setting it based on that
  // - You will want to use the existing setSpeed method within it
  // When this method is used, it returns a Command based on the information given to it at the time
  // that the method was called (which is essentially right away). Therefore, if parameters given to
  // this method will change (such as inputs from the driver), it will cause problems if they're just
  // normal parameters for the method. Don't worry about this for now (unless you are interested in
  // learning more about it), but we would want to do this a little differently for it to truly work.

  // TODO: Use the method you have now defined in RobotContainer (go to line 84).
}
