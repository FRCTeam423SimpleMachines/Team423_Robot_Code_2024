package frc.robot.commands.visionAim;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AimAtSpeaker extends Command {
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(4, 4);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.5, 0, 0.2, omegaConstraints);

    private final DriveSubsystem m_DriveSubsystem;
    private final VisionSubsystem m_VisionSubsystem;
    double omegaSpeed = 0.0;

    Pose2d targetPose = null;
    Rotation2d targetAngle = null;
    List<PhotonTrackedTarget> targets;
    PhotonTrackedTarget target = null;

    private GenericEntry targetAngleEntry = Shuffleboard.getTab("Auto 2").add("Target Angle",0.0).getEntry();
    private GenericEntry goalEntry = Shuffleboard.getTab("Auto 2").add("Goal",0.0).getEntry();
    private GenericEntry targetPoseEntry = Shuffleboard.getTab("Auto 2").add("Target Pose Rotation",0.0).getEntry();
    private GenericEntry bestTarID = Shuffleboard.getTab("Auto 2").add("Targeting ID", 0).getEntry();
    private GenericEntry numTars = Shuffleboard.getTab("Auto 2").add("Targets", 0).getEntry();

    /**
     * @param drive
     * @param vision
     */
    public AimAtSpeaker(VisionSubsystem vision, DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        m_VisionSubsystem = vision;

        omegaController.setTolerance(Units.degreesToRadians(5));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_DriveSubsystem);

        
    }

    @Override
    public void initialize() {
        Pose2d robotPose = m_DriveSubsystem.getPose();
        ChassisSpeeds robotVelocity = m_DriveSubsystem.getRobotRelativeSpeeds();

        omegaController.reset(robotPose.getRotation().getRadians(), -robotVelocity.omegaRadiansPerSecond);
        //omegaController.setGoal(0.0);

        //Stops the robot from getting stuck if it doesn't see an AprilTag
        if(!m_VisionSubsystem.hasTargets()){
            super.cancel();
        }
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_DriveSubsystem.getPose();
        

        if(m_VisionSubsystem.hasTargets()) {
            targets = m_VisionSubsystem.getTargets();
        }

        if(targets != null) {
            for (PhotonTrackedTarget tar : targets) {
                if(tar.getFiducialId()==4 || tar.getFiducialId()==8) {
                    target = tar;
                }
            }
        }
        
        
        if (m_VisionSubsystem.hasTargets() && target != null) {
            targetPose = m_VisionSubsystem.getTargetPose(VisionConstants.kTargetOffset, target);  
        }

        
        omegaController.setGoal(targetPose.getRotation().getRadians());
        omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());  
        

        if (omegaController.atGoal()) omegaSpeed = 0;

        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, -omegaSpeed));
        

        if (target != null && targetPose != null) {
            targetPoseEntry.setDouble(targetPose.getRotation().getDegrees());
            goalEntry.setDouble(Units.radiansToDegrees(omegaController.getGoal().position));
            bestTarID.setInteger(target.getFiducialId());
            numTars.setInteger(targets.size());
        }
    }

    @Override
    public boolean isFinished() {
        return omegaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}