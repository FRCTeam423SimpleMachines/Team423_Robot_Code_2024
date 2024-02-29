package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class Slide extends Command {

    ShooterIntakeSubsystem m_shooter;
    PIDController slideController;

    public Slide(ShooterIntakeSubsystem shooter) {
        slideController = new PIDController(0, 0, 0);
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        slideController.setSetpoint(ShooterConstants.kSlideDownPosition);
    }

    @Override
    public void execute() {
        m_shooter.setSlide(slideController.calculate(m_shooter.getSlide()));
    }
    
}
