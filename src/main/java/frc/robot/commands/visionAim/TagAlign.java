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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TagAlign extends Command {
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(4, 4);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.8, 0, 0, omegaConstraints);

    private final DriveSubsystem m_DriveSubsystem;
    private final VisionSubsystem m_VisionSubsystem;

    private GenericEntry targetAngleEntry = Shuffleboard.getTab("Auto").add("Target Angle",0.0).getEntry();
    private GenericEntry goalEntry = Shuffleboard.getTab("Auto").add("Goal",0.0).getEntry();
    private GenericEntry targetPoseEntry = Shuffleboard.getTab("Auto").add("Target Pose Rotation",0.0).getEntry();
    
    /**
     * @param drive
     * @param vision
     */
    public TagAlign(VisionSubsystem vision, DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        m_VisionSubsystem = vision;

        omegaController.setTolerance(Units.degreesToRadians(10));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = m_DriveSubsystem.getPose();
        ChassisSpeeds robotVelocity = m_DriveSubsystem.getRobotRelativeSpeeds();

        omegaController.reset(robotPose.getRotation().getRadians(), -robotVelocity.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_DriveSubsystem.getPose();
        Pose2d targetPose;
        Rotation2d targetAngle;
        
        if (m_VisionSubsystem.hasTargets()) {
            targetPose = m_VisionSubsystem.getTargetPose(VisionConstants.kTargetOffset);
            targetAngle = targetPose.minus(robotPose).getTranslation().getAngle();
            //omegaController.setGoal(robotPose.getRotation().plus(targetAngle).getRadians());
            omegaController.setGoal(targetAngle.getRadians());
        }

        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (omegaController.atGoal()) omegaSpeed = 0;

        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, omegaSpeed));

        // targetAngleEntry.setDouble(targetAngle.getDegrees());
        // targetPoseEntry.setDouble(targetPose.getRotation().getDegrees());
        goalEntry.setDouble(Units.radiansToDegrees(omegaController.getGoal().position));
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