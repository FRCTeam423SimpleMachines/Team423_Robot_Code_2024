package frc.robot.subsystems;

import java.util.Optional;

import javax.swing.JList.DropLocation;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class VisionSubsystem extends SubsystemBase{

     private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;

    private static final Pose3d ROBOT_TO_CAMERA = new Pose3d(
        Units.inchesToMeters(5.5), 
        Units.inchesToMeters(-4), 
        Units.inchesToMeters(18),
        new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180))
        );

    private PhotonPipelineResult results;
    private double m_latestLatency = 0.0;
    GenericEntry aimAngle;
    DriveSubsystem m_DriveSubsystem;
  
    public double getVisionTX(){
        return getLatestEstimatedRobotPose().getX();
    }

    public VisionSubsystem(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        m_camera = new PhotonCamera("Limelight");
        results = m_camera.getLatestResult(); 
    }
    
    
        public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        m_latestLatency = result.getLatencyMillis() / 1000.;

        boolean hasTarget = result.hasTargets();

        PhotonTrackedTarget target = null;

        if (hasTarget) {
            target = result.getBestTarget();
        }

        return target;
    }

    public Pose2d getLatestEstimatedRobotPose() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose.toPose2d();
            }
        }
        return new Pose2d();
    }

    public Pose2d getTargetPose() {
        PhotonTrackedTarget target = getBestTarget();
        Pose2d robotPose = m_DriveSubsystem.getPose();
        Transform3d offset = new Transform3d(ROBOT_TO_CAMERA.getTranslation(), ROBOT_TO_CAMERA.getRotation());

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Transform3d targetOffset = cameraToTarget.plus(offset);

            Pose3d pose = new Pose3d(robotPose);

            Pose3d scoringPose = pose.plus(targetOffset);

            // WARNING: The following code is scuffed. Please proceed with caution.
            Pose2d newPose = scoringPose.toPose2d();

            Rotation2d newRotation = Rotation2d.fromDegrees(newPose.getRotation().getDegrees() - 180.);

            Pose2d finalPose = new Pose2d(newPose.getTranslation(), newRotation).plus(
                    new Transform2d(
                            ROBOT_TO_CAMERA.getTranslation().toTranslation2d(),
                            ROBOT_TO_CAMERA.getRotation().toRotation2d()));
            return finalPose;
        }

        return robotPose;
    }

    public double getLatestLatency() {
        return m_latestLatency;
    }



    @Override
    public void periodic() {
        results = m_camera.getLatestResult(); 
        
        if (results.hasTargets()) {
            m_DriveSubsystem.addVisionPoseEstimate(getLatestEstimatedRobotPose(), Timer.getFPGATimestamp());
        }
    }

    
    
  }


















