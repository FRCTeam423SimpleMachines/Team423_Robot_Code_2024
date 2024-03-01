package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    private final CANSparkMax ClimbSparkMax;

    public ClimbSubsystem() {
        ClimbSparkMax = new CANSparkMax(ClimbConstants.kClimbMotoCANId, MotorType.kBrushless);
    }

    public void runClimb(double speed) {
        ClimbSparkMax.set(speed);
    }
    
}
