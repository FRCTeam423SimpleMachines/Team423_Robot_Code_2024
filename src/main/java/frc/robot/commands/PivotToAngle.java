package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class PivotToAngle extends Command{
    private PIDController pivotController;
    private ShooterIntakeSubsystem m_shooter;
    private double desiredAngle;
    private double output;

    public PivotToAngle(ShooterIntakeSubsystem shooter, double angle) {
        pivotController = new PIDController(1.3, 0, 0);
        m_shooter = shooter;
        desiredAngle = angle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        pivotController.setSetpoint(desiredAngle);
        pivotController.enableContinuousInput(0, 360);
        pivotController.setTolerance(5);
    }

    @Override
    public void execute() {
        output = (pivotController.calculate(m_shooter.getPitchDegrees()))/100;
        m_shooter.setPitch(output);
        SmartDashboard.putNumber("PID stuff", pivotController.getSetpoint());
        SmartDashboard.putNumber("PID output", output);
        m_shooter.setReady(false);
    }

    @Override
    public boolean isFinished() {
        return pivotController.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {
        m_shooter.setPitch(0);
        m_shooter.setReady(true);
    }

}
