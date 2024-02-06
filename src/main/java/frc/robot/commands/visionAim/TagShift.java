package frc.robot.commands.visionAim;


import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class TagShift extends Command{
    

    private PIDController strafeController = new PIDController(0.01, 0.0, 0);
    private SlewRateLimiter strafeSlewer = new SlewRateLimiter(3);



    

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
        strafeController.setTolerance(1);
        strafeController.setSetpoint(0);


        strafeSlewer.reset(0);
    
    } 


    

    @Override
    public void execute() {
        strafePID = strafeController.calculate(tx);
        tx = m_VisionSubsystem.getVisionTX();

            m_DriveSubsystem.drive(0, -strafePID, 0, false, false);
        }


    @Override
    public boolean isFinished(){
        return strafeController.atSetpoint();
    }
}