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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TagAlign extends Command {
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(8, 0, 0, omegaConstraints);

    private final DriveSubsystem m_DriveSubsystem;
    private final VisionSubsystem m_VisionSubsystem;



    
    
    


    
    /**
     * @param drive
     * @param vision
     */
    public TagAlign(VisionSubsystem vision, DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        m_VisionSubsystem = vision;

        omegaController.setTolerance(Units.degreesToRadians(0.5));
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
        Pose2d targetPose = m_VisionSubsystem.getTargetPose();

        Rotation2d targetAngle = targetPose.getRotation();

        omegaController.setGoal(targetAngle.getRadians());
        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        

        if (omegaController.atGoal()) omegaSpeed = 0;

        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
        
    }
}