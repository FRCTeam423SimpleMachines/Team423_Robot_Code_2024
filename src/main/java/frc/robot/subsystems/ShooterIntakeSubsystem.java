package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterIntakeSubsystem extends SubsystemBase{
    
    private final CANSparkMax m_LeftShooterMoter;
    private final CANSparkMax m_RightShooterMoter;
    private final CANSparkMax m_PitchSparkMax;
    private final RelativeEncoder m_LeftEncoder;
    private final RelativeEncoder m_RightEncoder;
    private final RelativeEncoder m_PitchEncoder;
    private final AbsoluteEncoder m_PitchAbsoluteEncoder;

 

    public ShooterIntakeSubsystem() {
        m_LeftShooterMoter = new CANSparkMax(ShooterConstants.kLeftMotorCANId, MotorType.kBrushless);
        m_RightShooterMoter = new CANSparkMax(ShooterConstants.kRightMotorCANId, MotorType.kBrushless);
        m_PitchSparkMax = new CANSparkMax(ShooterConstants.kPitchMotorCANId, MotorType.kBrushless);
        m_LeftEncoder = m_LeftShooterMoter.getEncoder();
        m_RightEncoder = m_RightShooterMoter.getEncoder();
        m_PitchEncoder = m_PitchSparkMax.getEncoder();
        m_PitchAbsoluteEncoder = m_PitchSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void runShooter(double lSpeed, double rSpeed) {
        m_LeftShooterMoter.set(lSpeed);
        m_RightShooterMoter.set(rSpeed);
    }

    public void setShooterVolt(double lVoltage, double rVoltage ) {
        m_LeftShooterMoter.setVoltage(lVoltage);
        m_LeftShooterMoter.setVoltage(rVoltage);
    }

    public void setPitchSpeed(double speed) {
        m_PitchSparkMax.set(speed);
    }

    public double getLeftRPM() {
        return m_LeftEncoder.getVelocity();
    }

    public double getRightRPM() {
        return m_RightEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left RPM", getLeftRPM());
        SmartDashboard.putNumber("Right RPM", getRightRPM());
        SmartDashboard.putString("Shooter Command", getCommandName());

    }
    //SparkMAX -> Relative Encoder -> RPM -> PID -> Motor

    public String getCommandName() {
        if(getCurrentCommand()!=null){
            return getCurrentCommand().getName();
        }
        return "None";
    }


}
