package frc.robot.commands.visionAim;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class TagShift extends Command{
    

    private PIDController strafeController = new PIDController(0.05, 0.0, 0);

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
        strafeController.setTolerance(5);
    }
    

    

    @Override
    public void execute() {
       
        strafeSpeed = strafeController.calculate(m_VisionSubsystem.getVisionYaw());   
        
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

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}