package frc.robot.commands.visionAim;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AimAtPose extends Command {
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(8, 0, 0, omegaConstraints);

    private final DriveSubsystem m_DriveSubsystem;

    private Pose2d targetPose;

    

    /**
     * Aims at the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param m_DriveSubsystem
     * @param targetPoseSupplier
     */
    public AimAtPose(DriveSubsystem drive, Pose2d pose) {
        m_DriveSubsystem = drive;
        targetPose = pose;

        omegaController.setTolerance(Units.degreesToRadians(5));
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

        Rotation2d targetAngle = targetPose.minus(robotPose).getTranslation().getAngle();

        omegaController.setGoal(robotPose.getRotation().plus(targetAngle).getRadians());

        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (omegaController.atGoal()) omegaSpeed = 0;

        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, omegaSpeed));

        SmartDashboard.putBoolean("At Goal", omegaController.atGoal());
        SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Goal", Units.radiansToDegrees(omegaController.getGoal().position));
    }

    @Override
    public boolean isFinished() {
        return omegaController.atGoal();
    }
}