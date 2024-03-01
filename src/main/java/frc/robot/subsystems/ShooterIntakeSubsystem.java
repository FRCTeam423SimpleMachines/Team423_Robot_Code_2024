package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;

public class ShooterIntakeSubsystem extends SubsystemBase{
    
    private final CANSparkMax m_LeftShooterMoter;
    private final CANSparkMax m_RightShooterMoter;
    private final CANSparkMax m_PitchSparkMax;
    private final CANSparkMax m_SlideSparkMax;
    private final CANSparkMax m_IntakeSparkMax;
    private final RelativeEncoder m_LeftEncoder;
    private final RelativeEncoder m_RightEncoder;
    private final DutyCycleEncoder m_PitchEncoder;
    private final RelativeEncoder m_SlideEncoder;
    private final RelativeEncoder m_IntakeEncoder;
    //private final AbsoluteEncoder m_PitchAbsoluteEncoder;
    private final DigitalInput sensorInput;
    private final DigitalInput sensorInput2;

 

    public ShooterIntakeSubsystem() {
        m_LeftShooterMoter = new CANSparkMax(ShooterConstants.kLeftMotorCANId, MotorType.kBrushless);
        m_RightShooterMoter = new CANSparkMax(ShooterConstants.kRightMotorCANId, MotorType.kBrushless);
        m_PitchSparkMax = new CANSparkMax(ShooterConstants.kPitchMotorCANId, MotorType.kBrushless);
        m_SlideSparkMax = new CANSparkMax(ShooterConstants.kSlideMotorCANId, MotorType.kBrushless);
        m_IntakeSparkMax = new CANSparkMax(IntakeConstants.kIntakeMotorCANId, MotorType.kBrushless);
        m_LeftEncoder = m_LeftShooterMoter.getEncoder();
        m_RightEncoder = m_RightShooterMoter.getEncoder();
        m_PitchEncoder = new DutyCycleEncoder(3);
        m_SlideEncoder = m_SlideSparkMax.getEncoder();
        //m_PitchAbsoluteEncoder = m_PitchSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_IntakeEncoder = m_IntakeSparkMax.getEncoder();
        sensorInput = new DigitalInput(IntakeConstants.kDIOInputID1);
        sensorInput2 = new DigitalInput(IntakeConstants.kDIOInputID2);
        
    }

    public void runShooter(double lpow, double rpow) {
        m_LeftShooterMoter.set(lpow);
        m_RightShooterMoter.set(lpow);
    }

    public void setShooterVolt(double lVoltage, double rVoltage ) {
        m_LeftShooterMoter.setVoltage(lVoltage);
        m_LeftShooterMoter.setVoltage(rVoltage);
    }

    public void setPitch(double pow) {
        m_PitchSparkMax.set(pow);
    }

    public void setSlide(double pow) {
        m_SlideSparkMax.set(pow);
    }

    public void setIntake(double pow) {
        m_IntakeSparkMax.set(pow);
    }

    public double getIntake() {
        return m_IntakeEncoder.getVelocity();
    }

    public double getLeftRPM() {
        return m_LeftEncoder.getVelocity();
    }

    public double getRightRPM() {
        return m_RightEncoder.getVelocity();
    }

    public double getPitchDegrees() {
        return m_PitchEncoder.getAbsolutePosition()*360;
    }

    public double getSlide() {
        return m_SlideEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left RPM", getLeftRPM());
        SmartDashboard.putNumber("Right RPM", getRightRPM());
        SmartDashboard.putString("Shooter Command", getCommandName());
        SmartDashboard.putNumber("Pivot Angle", getPitchDegrees());
    }
    //SparkMAX -> Relative Encoder -> RPM -> PID -> Motor

    public String getCommandName() {
        if(getCurrentCommand()!=null){
            return getCurrentCommand().getName();
        }
        return "None";
    }
}