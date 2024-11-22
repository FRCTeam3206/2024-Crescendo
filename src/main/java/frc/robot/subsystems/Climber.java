package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
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
    this.log("Climb Speed", speed);
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

  // Extra challenge (optional; probably difficult, but helpful to understand if you're interested):
  // Modify the above method to take a Supplier as its argument instead of just a value.
  // This allows the value you use to change after the method is used to create the Command, which would
  // be important in order for its values to be based on driver inputs. A Supplier is a type of
  // FunctionalInterface. The "run" method takes a FunctionalInterface as an argument (in that case,
  // it needs a Runnable). FunctionalInterfaces are sort of like classes that only one thing (a method
  // that hasn't been defined), which objects can be created of by giving it the definition for that
  // method (this isn't a fully accurate definition, but it mostly conveys the idea behind them). The
  // definition of that method is given as a "lambda", which is in the form of () -> {} Essentially,
  // the parameters of the method you're defining go in the parenthesis and the action goes in the 
  // curly brackets. For Runnable, there aren't any parameters, so the parenthesis stay empty, and it
  // doesn't return anything; it only does action(s). A Supplier returns a value, but it doesn't take any
  // parameters. For the case of this particular challenge, you will want to have a Supplier so that it
  // can be defined based on a method that will give different values (for example, a method that gets
  // input from a joystick) and then in the Runnable that you give to the run method, you will want to
  // get the value from the Supplier that is a parameter. A more fully accurate explanation of what a
  // FunctionalInterface is: in Java, you can create classes from other classes. To do that, in the
  // line where you say "class", you say "MyClass extends OtherClass" (for example, Blueberry extends
  // Fruit). The class that extends another class (for example, MyClass or Blueberry) is a subclass of
  // the other class (for example, OtherClass or Fruit). (To be continued)
}
