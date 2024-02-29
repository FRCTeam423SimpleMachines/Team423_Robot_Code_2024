package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class RunIntake extends Command {

    PIDController intakeController;
    ShooterIntakeSubsystem m_Intake;
    SimpleMotorFeedforward intakeFeedforward;
    double speed;
    
    public RunIntake(ShooterIntakeSubsystem shooter, double speed) {
        m_Intake = shooter; 
        intakeController = new PIDController(0.1, 0, 0);
        intakeFeedforward = new SimpleMotorFeedforward(0.0, 0.165);
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        intakeController.setTolerance(10);
        intakeController.setSetpoint(speed);
    }

    public void execute() {
        m_Intake.setIntake(intakeFeedforward.calculate(speed) + intakeController.calculate(m_Intake.getIntake()));
    }
}
