package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class ShooterIdle extends Command{
    
    ShooterIntakeSubsystem m_shooter;

    public ShooterIdle(ShooterIntakeSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.runShooter(0.0, 0.0);
        m_shooter.setIntake(0.0);
    }

    
}