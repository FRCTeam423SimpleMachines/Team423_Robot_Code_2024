package frc.robot.commands.visionAim;


import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class TagShift extends Command{
    

    private PIDController strafeController = new PIDController(0.05, 0.0, 0);
    private SlewRateLimiter strafeSlewer = new SlewRateLimiter(3);

    // double tx;
    // double ta;

    double strafeSpeed;
    
    Pose2d targetPose = null;

    private GenericEntry goal = Shuffleboard.getTab("Auto").add("Setpoint",0.0).getEntry();
    private GenericEntry targetPoseY = Shuffleboard.getTab("Auto").add("Target Pose Y",0.0).getEntry();

    VisionSubsystem m_VisionSubsystem;
    DriveSubsystem m_DriveSubsystem;
    
    public TagShift(VisionSubsystem vision, DriveSubsystem drive){
        strafeController.setSetpoint(0);
        m_VisionSubsystem = vision;
        m_DriveSubsystem = drive;
        addRequirements(vision, drive);
    }

    @Override
    public void initialize() {
        strafeController.setTolerance(0.1);
        strafeController.setSetpoint(0);


        strafeSlewer.reset(0);
    
    } 


    

    @Override
    public void execute() {
        //strafeSpeed = strafeController.calculate(tx);
        Pose2d robotPose = m_DriveSubsystem.getPose();
        
       
        if (m_VisionSubsystem.hasTargets()) {
            targetPose = m_VisionSubsystem.getTargetPose(VisionConstants.kTargetOffset);
            //strafeController.setSetpoint(targetPose.getY());
            strafeSpeed = strafeController.calculate(targetPose.getY());   
        }

        //tx = m_VisionSubsystem.getVisionTX();

        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0, strafeSpeed, 0));

        if(targetPose != null) {
            targetPoseY.setDouble(targetPose.getY());
        }
        goal.setDouble(strafeController.getSetpoint());
    }


    @Override
    public boolean isFinished(){
        return strafeController.atSetpoint();
    }
}