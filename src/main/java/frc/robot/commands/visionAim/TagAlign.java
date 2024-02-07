package frc.robot.commands.visionAim;


import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class TagAlign extends Command{
    
    private static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(4, 4);
    private ProfiledPIDController angleController = new ProfiledPIDController(0.1, 0, 0.0, angleConstraints);

    
    double tx;
    double ta;
    

    VisionSubsystem m_VisionSubsystem;
    DriveSubsystem m_DriveSubsystem;
    
    public TagAlign(VisionSubsystem vision, DriveSubsystem drive){
        m_VisionSubsystem = vision;
        m_DriveSubsystem = drive;
        addRequirements(vision, drive);
    }

    @Override
    public void initialize() {
        angleController.setGoal(0);
       /*  angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(3);*/
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Math.toRadians(3));
    } 


    

    @Override
    public void execute() {
       // tx = m_VisionSubsystem.getVisionTX();
        double angularCorrection = angleController.calculate(
          m_DriveSubsystem.getPose().getRotation().getRadians()
        );
        double angularSpeed = angleController.getSetpoint().velocity + angularCorrection;
        //double angularSpeed = angleController.calculate(tx);
        m_DriveSubsystem.driveRobotRelative( new ChassisSpeeds(0.0, 0.0, angularSpeed));
    }


    @Override
    public boolean isFinished(){
        return angleController.atGoal();
    }
}