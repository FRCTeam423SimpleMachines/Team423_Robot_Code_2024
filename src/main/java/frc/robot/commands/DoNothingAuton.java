package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DoNothingAuton extends Command{
    
    DriveSubsystem m_drive;

    public DoNothingAuton(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.drive(0.0, 0.0,0.0,true, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}