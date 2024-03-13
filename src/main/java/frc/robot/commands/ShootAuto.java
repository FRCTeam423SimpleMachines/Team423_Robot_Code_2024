// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterIntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterIntakeSubsystem m_ShooterSubsystem;
  private double RPM;
  private PIDController leftSpeedController = new PIDController(0.01, 0.0, 0.0);
  private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(0.0, 0.165);
  private PIDController rightSpeedController = new PIDController(0.01, 0.0, 0.0);
  private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(0.0, 0.165);
  private double time = -1;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootAuto(ShooterIntakeSubsystem shooter) {
    m_ShooterSubsystem = shooter;
    RPM = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftSpeedController.setSetpoint(RPM);
    leftSpeedController.setTolerance(100);
    rightSpeedController.setSetpoint(RPM);
    rightSpeedController.setTolerance(100);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftMeasuredRPM = m_ShooterSubsystem.getLeftRPM();
    double rightMeasuredRPM = m_ShooterSubsystem.getRightRPM();
    double leftDesiredPower = (1);//leftFeedforward.calculate(RPM) + leftSpeedController.calculate(leftMeasuredRPM))/1000;
    double rightDesiredPower = -(1);//rightFeedforward.calculate(RPM) + rightSpeedController.calculate(rightMeasuredRPM))/1000;
    m_ShooterSubsystem.runShooter(leftDesiredPower, rightDesiredPower);
    
    if(leftMeasuredRPM > 5000){
        m_ShooterSubsystem.setIntake(-1);
        if(time == -1) {
          time = Timer.getFPGATimestamp();
        }
    }
  }

  @Override
  public void end(boolean isInterrupted) {
    m_ShooterSubsystem.runShooter(0, 0);
    m_ShooterSubsystem.setIntake(0);
  }

  @Override
  public boolean isFinished() {
    //return (Timer.getFPGATimestamp() - time > 1);
    return m_ShooterSubsystem.getOptic1();
  }

  
}