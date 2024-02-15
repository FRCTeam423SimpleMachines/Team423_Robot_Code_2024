package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    private final CANSparkMax m_LeftSparkMax;
    private final CANSparkMax m_RightSparkMax;
    private final RelativeEncoder m_LeftEncoder;
    private final RelativeEncoder m_RightEncoder;
    private final PIDController m_leftShooterController = new PIDController(0.1, 0, 0);
    private final PIDController m_rightShooterController = new PIDController(0.1, 0, 0);
    private double targetPitch; 
 

    public ShooterSubsystem() {
        m_LeftSparkMax = new CANSparkMax(20, MotorType.kBrushless);
        m_RightSparkMax = new CANSparkMax(21, MotorType.kBrushless);
        m_LeftEncoder = m_LeftSparkMax.getEncoder();
        m_RightEncoder = m_RightSparkMax.getEncoder();
    }

    public void runShooter(int desiredRPM) {
        m_LeftSparkMax.set(m_leftShooterController.calculate(m_LeftEncoder.getVelocity(), desiredRPM));
        m_RightSparkMax.set(m_rightShooterController.calculate(m_RightEncoder.getVelocity(), -desiredRPM));
    }



    @Override
    public void periodic() {



    }
    //SparkMAX -> Relative Encoder -> RPM -> PID -> Motor


















}
