package frc.robot.commands.visionAim;


import java.lang.annotation.Target;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

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
        
    }
    

    

    @Override
    public void execute() {
       
        strafeSpeed = strafeController.calculate(m_VisionSubsystem.getVisionYaw());   
        
        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0, -strafeSpeed, 0));

        if(targetPose != null) {
            targetPoseY.setDouble(targetPose.getY());
        }
        goal.setDouble(strafeController.getSetpoint());
    }


    @Override
    public boolean isFinished(){
        return strafeController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}