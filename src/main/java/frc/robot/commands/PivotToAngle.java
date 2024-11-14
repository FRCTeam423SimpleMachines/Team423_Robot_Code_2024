package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToAngle extends Command{

    private PIDController pivotController;
    private PivotSubsystem m_pivot;
    private double desiredAngle;
    private double output;
    

    public PivotToAngle(PivotSubsystem pivot, double angle) {
        pivotController = new PIDController(1.3, 0, 0);
        m_pivot = pivot;
        desiredAngle = angle;
        addRequirements(pivot);
        if(angle == 195){
            m_pivot.pivotPos.setString("Near Verticle");
        } else if(angle == 217) {
            m_pivot.pivotPos.setString("Side");
        } else if(angle == 240) {
            m_pivot.pivotPos.setString("Source");
        } else if(angle == 220) {
           m_pivot.pivotPos.setString("Straight");
        } 
    }

    @Override
    public void initialize() {
        pivotController.setSetpoint(desiredAngle);
        pivotController.enableContinuousInput(0, 360);
        pivotController.setTolerance(1);
    }

    @Override
    public void execute() {
        output = (pivotController.calculate(m_pivot.getPitchDegrees()))/100;
        m_pivot.setPitch(output);
        SmartDashboard.putNumber("PID stuff", pivotController.getSetpoint());
    }

}
