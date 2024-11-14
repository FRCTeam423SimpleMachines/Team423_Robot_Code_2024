// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

public class PivotSubsystem extends SubsystemBase {

    private final CANSparkMax m_PitchSparkMax;
    private final DutyCycleEncoder m_PitchEncoder;
    public GenericEntry pivotPos = Shuffleboard.getTab("Pivot").add("Pivot Pos", "").getEntry();  

    /** Creates a new ExampleSubsystem. */
    public PivotSubsystem() {
        m_PitchSparkMax = new CANSparkMax(PivotConstants.kPivotMotorCANId, MotorType.kBrushless);
        m_PitchEncoder = new DutyCycleEncoder(3);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        SmartDashboard.putNumber("Pivot Angle", getPitchDegrees());
        SmartDashboard.putNumber("Pivot Power", m_PitchSparkMax.get());
    }

    public void setPitch(double pow) {
        m_PitchSparkMax.set(pow);
    }

    public double getPitchDegrees() {
        return m_PitchEncoder.getAbsolutePosition()*360;
    }

    public double getRealPitch() {
        return getPitchDegrees() + PivotConstants.kPitchEncoderOffset;
    }

    public double getShooterHeight() {
       return Units.inchesToMeters(22.5)*Math.sin(Units.degreesToRadians(90-getRealPitch()));
    }

    public double getShooterX() {
       return Units.inchesToMeters(22.5)*Math.cos(Units.degreesToRadians(90-getRealPitch()));
    }
}
