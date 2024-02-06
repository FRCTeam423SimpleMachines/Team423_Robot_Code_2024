package frc.robot.subsystems;

import javax.swing.JList.DropLocation;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.visionAim.TagAlign;
import frc.robot.commands.visionAim.TagAlign;

public class VisionSubsystem extends SubsystemBase{
    
    private double tx;
    private double ta;
    private double ty;
    GenericEntry aimAngle;
    DriveSubsystem m_DriveSubsystem;
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");

    public VisionSubsystem(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
    }
    public double getVisionTX() {
         tx = LimelightHelpers.getTX("");
        return tx;
    }
     public double getVisionTY() {
         ty = LimelightHelpers.getTY("");
        return ty;
    }
     public double getVisionTA() {
         ta = LimelightHelpers.getTA("");
        return ta;
    }


    @Override
    public void periodic() {
        tx = LimelightHelpers.getTX("");
        
        m_DriveSubsystem.addVisionPoseEstimate(llresults.targetingResults.getBotPose2d(),Timer.getFPGATimestamp());

    }

    
    
  }


















