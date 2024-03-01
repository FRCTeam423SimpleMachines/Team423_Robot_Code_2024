package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MathUtils;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;



public class VisionSubsystem extends SubsystemBase{

    private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;
    private PhotonTrackedTarget m_target;
    private GenericEntry targetPoseX = Shuffleboard.getTab("Vision").add("Target Pose X", 0.0).getEntry();
    private GenericEntry targetPoseY = Shuffleboard.getTab("Vision").add("Target Pose Y", 0.0).getEntry();
    private GenericEntry targetPoseRot = Shuffleboard.getTab("Vision").add("Target Pose Rot", 0.0).getEntry();
    private GenericEntry yaw = Shuffleboard.getTab("Vision").add("Yaw", 0.0).getEntry();
    private GenericEntry bestTarID = Shuffleboard.getTab("Vision").add("Best ID", 0).getEntry();
    private PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult results;
    private double m_latestLatency = 0.0;
    GenericEntry aimAngle;
    
    DriveSubsystem m_DriveSubsystem;
  
    public double getVisionYaw(){
        if(m_target != null) {
            return m_target.getYaw();
        }
        return 0.0;
    }

    public VisionSubsystem(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        m_camera = new PhotonCamera("Limelight");
        results = m_camera.getLatestResult(); 
        m_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        Transform3d robotToCam = new Transform3d(VisionConstants.kRobotToCamera.getTranslation(),VisionConstants.kRobotToCamera.getRotation());
        poseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCam);    
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

    public List<PhotonTrackedTarget> getTargets() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        m_latestLatency = result.getLatencyMillis() / 1000.;

        boolean hasTarget = result.hasTargets();

        List<PhotonTrackedTarget> targets = null;

        if (hasTarget) {
            targets = result.getTargets();
        }

        return targets;
    }

    public Pose2d getLatestEstimatedRobotPose() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = new Transform3d(target.getBestCameraToTarget().getTranslation(), 
                new Rotation3d(
                    Units.degreesToRadians(target.getSkew()), 
                    Units.degreesToRadians(target.getPitch()), 
                    Units.degreesToRadians(target.getYaw())
                )
            );

            Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose.toPose2d();
            }
        }
        return new Pose2d();
    }

    public Pose2d getTargetPose(Transform3d offset, PhotonTrackedTarget target) {
        
        Pose2d robotPose = m_DriveSubsystem.getPose();
        

        if (target != null) {
            Transform3d cameraToTarget = new Transform3d(target.getBestCameraToTarget().getTranslation(), 
                new Rotation3d(
                    Units.degreesToRadians(target.getSkew()), 
                    Units.degreesToRadians(target.getPitch()), 
                    Units.degreesToRadians(target.getYaw())
                )
            );

            Transform3d targetOffset = cameraToTarget.plus(offset);

            Pose3d pose = new Pose3d(robotPose);

            Pose3d scoringPose = pose.plus(targetOffset);

            // WARNING: The following code is scuffed. Please proceed with caution.
            Pose2d newPose = scoringPose.toPose2d();

            Pose2d finalPose = newPose.plus(
                    new Transform2d(
                            VisionConstants.kRobotToCamera.getTranslation().toTranslation2d(),
                            VisionConstants.kRobotToCamera.getRotation().toRotation2d()));
            return finalPose;
        }

        return robotPose;
    }

    private void addVisionPoseEstimate(EstimatedRobotPose estimate) {
        if (!isValidPose(estimate.estimatedPose)) return;

        var estimatedPose = estimate.estimatedPose.toPose2d();

        double averageDistance = 0;

        for (PhotonTrackedTarget target : estimate.targetsUsed) {
            averageDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        averageDistance /= estimate.targetsUsed.size();

        m_DriveSubsystem.addVisionPoseEstimate(
                estimatedPose, estimate.timestampSeconds, calculateVisionStdDevs(averageDistance));
    }

    private boolean isValidPose(Pose3d pose) {
        boolean isWithinField = MathUtils.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
                && MathUtils.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5)
                && MathUtils.isInRange(pose.getZ(), 0, 5);

        boolean isNearRobot = m_DriveSubsystem
                        .getPose()
                        .getTranslation()
                        .getDistance(pose.getTranslation().toTranslation2d())
                < 1.4;

        return isWithinField && isNearRobot;
    }

    private Matrix<N3, N1> calculateVisionStdDevs(double distance) {
        var translationStdDev = 0.3 * Math.pow(distance, 2);
        var rotationStdDev = 0.9 * Math.pow(distance, 2);

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    public double getLatestLatency() {
        return m_latestLatency;
    }

    public boolean hasTargets() {
       return results.hasTargets();
    }

    @Override
    public void periodic() {
        results = m_camera.getLatestResult(); 
        m_target = results.getBestTarget();
        
        
        var estimatedPose = poseEstimator.update();
        
        if(estimatedPose.isPresent()) {
            addVisionPoseEstimate(estimatedPose.get());  
        }

        targetPoseX.setDouble(getTargetPose(VisionConstants.kTargetOffset, results.getBestTarget()).getX());
        targetPoseY.setDouble(getTargetPose(VisionConstants.kTargetOffset, results.getBestTarget()).getY());
        targetPoseRot.setDouble(getTargetPose(VisionConstants.kTargetOffset, results.getBestTarget()).getRotation().getDegrees());
        yaw.setDouble(getVisionYaw());
        
        bestTarID.setInteger(getTargetID());
    }

    public int getTargetID() {
        if(hasTargets()) {
            return m_target.getFiducialId();
        }
        return -1;
    }

  }