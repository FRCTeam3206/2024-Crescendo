package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    CANSparkMax leftClimber = new CANSparkMax(ClimberConstants.kLeftClimberCANId, MotorType.kBrushless);
    CANSparkMax rightClimber = new CANSparkMax(ClimberConstants.kRightClimberCANId, MotorType.kBrushless);
    public Climber() {}

    public void setSpeed(double speed) {
        leftClimber.set(speed);
        rightClimber.set(-speed);
    }
}
