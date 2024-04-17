package frc.robot.commands.visionAim;

import java.lang.reflect.Field;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DistanceAim extends Command {
    PivotSubsystem m_pivot;
    VisionSubsystem m_vison;
    PIDController pivotController;
    
    public DistanceAim(PivotSubsystem pivot, VisionSubsystem vison) {
        m_pivot = pivot;
        m_vison = vison;
        addRequirements(pivot, vison);
    }

    @Override
    public void initialize() {
        pivotController = new PIDController(1.3, 0, 0);
        //pivotController.setTolerance(2);
    }

    @Override
    public void execute() {
        double xOffset = Units.inchesToMeters(7.625)+m_pivot.getShooterX();
        pivotController.setSetpoint(m_vison.getPivotAngle(m_vison.getSpeakerDistance()-xOffset, FieldConstants.speakerHeight, PivotConstants.pivotHieght+m_pivot.getShooterHeight(), 26.72));
        m_pivot.setPitch(pivotController.calculate(m_pivot.getRealPitch()));
    }
}
