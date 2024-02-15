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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class TagShift extends Command{
    

    private PIDController strafeController = new PIDController(0.01, 0.0, 0);
    private SlewRateLimiter strafeSlewer = new SlewRateLimiter(3);
    PathPlannerTrajectory PathPlanner;
    PhotonCamera camera = new PhotonCamera("Limelight");
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    
    Rotation2d neededRotation2d;

    double tx;
    double ta;

    double strafePID;
    

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
        strafePID = strafeController.calculate(tx);
        tx = m_VisionSubsystem.getVisionTX();
        Transform3d camToTarget = target.getBestCameraToTarget();
        neededRotation2d = PhotonUtils.getYawToPose(m_DriveSubsystem.getPose(), null);
        
    }
        
        

    @Override
    public boolean isFinished(){
        return strafeController.atSetpoint();
    }
}