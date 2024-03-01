// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterIntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootAtSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterIntakeSubsystem m_ShooterSubsystem;
  private double RPM;
  private PIDController leftSpeedController = new PIDController(0.01, 0.0, 0.0);
  private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(0.0, 0.165);
  private PIDController rightSpeedController = new PIDController(0.00000000, 0.0, 0.0);
  private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(0.0, 0.165);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootAtSpeed(ShooterIntakeSubsystem shooter, double speed) {
    m_ShooterSubsystem = shooter;
    RPM = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftSpeedController.setSetpoint(RPM);
    leftSpeedController.setTolerance(10);
    rightSpeedController.setSetpoint(RPM);
    rightSpeedController.setTolerance(10);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftMeasuredRPM = m_ShooterSubsystem.getLeftRPM();
    double rightMeasuredRPM = m_ShooterSubsystem.getRightRPM();
    double leftDesiredPower = (leftFeedforward.calculate(RPM) + leftSpeedController.calculate(leftMeasuredRPM))/1000;
    double rightDesiredPower = (rightFeedforward.calculate(RPM) + rightSpeedController.calculate(rightMeasuredRPM))/1000;
    m_ShooterSubsystem.runShooter(leftDesiredPower, -rightDesiredPower);

    SmartDashboard.putNumber("Desired Power LEFT", leftDesiredPower);
    SmartDashboard.putNumber("Desired Power RIhGT", rightDesiredPower);
  }
}
