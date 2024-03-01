package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class RunIntake extends Command {

    ShooterIntakeSubsystem m_Intake;
    double speed;
    
    public RunIntake(ShooterIntakeSubsystem shooter, double speed) {
        m_Intake = shooter; 
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_Intake.setIntake(speed);
    }


}
