// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootAtSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_ShooterSubsystem;
  private double RPM;
  private PIDController speedController = new PIDController(0.01, 0.0, 0.0);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.165);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootAtSpeed(ShooterSubsystem shooter, double speed) {
    m_ShooterSubsystem = shooter;
    RPM = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedController.setSetpoint(RPM);
    speedController.setTolerance(10);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measuredRPM = m_ShooterSubsystem.getLeftRPM();
    double leftDesiredPower = (feedforward.calculate(RPM) + speedController.calculate(measuredRPM))/1000;
    m_ShooterSubsystem.runShooter(leftDesiredPower, 0.0);

    SmartDashboard.putNumber("Desired Power", leftDesiredPower);
  }
}
