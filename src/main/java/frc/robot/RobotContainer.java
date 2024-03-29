// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Autos;
import frc.robot.commands.DoNothingAuton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.visionAim.AimAtPose;
import frc.robot.commands.visionAim.AimAtSpeaker;
import frc.robot.commands.visionAim.TagAlign;
import frc.robot.commands.visionAim.TagShift;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.FieldConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_DriveSubsystem);

  private final CommandJoystick m_driverController1 = new CommandJoystick(ControlConstants.kControllerPort1); 
  private final CommandJoystick m_driverController2 = new CommandJoystick(ControlConstants.kControllerPort2); 

  SendableChooser<Command> m_chooser; 


  private SlewRateLimiter slewY = new SlewRateLimiter(2);
  private SlewRateLimiter slewX = new SlewRateLimiter(2);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Register named commads for pathplanner 
    NamedCommands.registerCommand("AimAtSpeaker", new AimAtSpeaker(m_VisionSubsystem, m_DriveSubsystem));
    //NamedCommands.registerCommand("Shoot", new AimAndShoot()); //Whatever makes the shooter shoot and aim
    //NamedCommands.registerCommand("PickUpInit", new ParallelCommandGroup(null)); //Command group to set the robot up to pick up rings during auton

    m_chooser = AutoBuilder.buildAutoChooser();
    
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    // Configure the trigger bindings
    configureBindings();

    m_DriveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_DriveSubsystem.drive(
          -0.5*slewY.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftYAxis), 0.15)) ,
          -0.5*slewX.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftXAxis), 0.15)) ,
          -0.5*(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kRightXAxis), 0.15)),
          true, true), m_DriveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    Trigger xButton1 = m_driverController1.button(ControlConstants.kXButton);
    Trigger rBumper1 = m_driverController1.button(ControlConstants.kRightBumper);
    Trigger aButton1 = m_driverController1.button(ControlConstants.kAButton);

    Trigger yButton1 = m_driverController1.button(ControlConstants.kYButton);

    Trigger bButton2 = m_driverController2.button(ControlConstants.kBButton);
    Trigger xButton2 = m_driverController2.button(ControlConstants.kXButton);


    xButton1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.setX(), 
        m_DriveSubsystem));


    yButton1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.resetDrive(),
        m_DriveSubsystem));

    aButton1.onTrue(
      new AimAtSpeaker(m_VisionSubsystem, m_DriveSubsystem));

    rBumper1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.drive(
          -slewY.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftYAxis), 0.15)) ,
          -slewX.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftXAxis), 0.15)) ,
          -(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kRightXAxis), 0.15)),
          true, true), m_DriveSubsystem));

    bButton2.onTrue(
      new TagAlign(m_VisionSubsystem, m_DriveSubsystem));  
      
    xButton2.onTrue(
      new TagShift(m_VisionSubsystem, m_DriveSubsystem));

    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public double squareInput(double x){
    if (x > 0){
      return Math.pow(x, 2);
    } else {
      return -Math.pow(x,2);
    }
  }
}
