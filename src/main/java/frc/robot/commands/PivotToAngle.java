package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class PivotToAngle extends Command{
    private PIDController pivotController;
    private ShooterIntakeSubsystem m_shooter;
    private double desiredAngle;

    public PivotToAngle(ShooterIntakeSubsystem shooter, double angle) {
        pivotController = new PIDController(0, 0, 0);
        m_shooter = shooter;
        desiredAngle = angle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        pivotController.setSetpoint(desiredAngle);
        pivotController.enableContinuousInput(0, 360);
        pivotController.setTolerance(3);
    }

    @Override
    public void execute() {
        m_shooter.setIntakeSpeed(pivotController.calculate(m_shooter.getPitch()));
    }

    @Override
    public boolean isFinished() {
        return pivotController.atSetpoint();
    }

}
