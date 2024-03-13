package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivot extends Command {


    private PIDController pivotController;
    private PivotSubsystem m_pivot;
    private double desiredAngle;
    private double output;

    public AutoPivot(PivotSubsystem pivot, double angle) {
        pivotController = new PIDController(1.3, 0, 0);
        m_pivot = pivot;
        desiredAngle = angle;
        addRequirements(pivot);
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
     }

    @Override
    public void end(boolean isInterrupted) {
        m_pivot.setPitch(0);
    }

    @Override
    public boolean isFinished() {
        return pivotController.atSetpoint();
    }

}


