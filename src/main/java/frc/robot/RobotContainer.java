// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.visionAim.AimAtSpeaker;
import frc.robot.commands.visionAim.TagAlign;
import frc.robot.commands.AutoPivot;
import frc.robot.commands.EndShoot;
import frc.robot.commands.LoadIntake;
import frc.robot.commands.PivotToAngle;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAtSpeed;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.visionAim.TagShift;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_DriveSubsystem);
  private final ShooterIntakeSubsystem m_ShooterSubsystem = new ShooterIntakeSubsystem();
  private final ClimbSubsystem m_Climb = new ClimbSubsystem(); 
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
  private final Field2d field;
  
  private final CommandJoystick m_driverController1 = new CommandJoystick(ControlConstants.kControllerPort1); 
  private final CommandJoystick m_driverController2 = new CommandJoystick(ControlConstants.kControllerPort2); 

  SendableChooser<Command> m_chooser; 

  private double pivotTune = 220;

  private SlewRateLimiter slewY = new SlewRateLimiter(2);
  private SlewRateLimiter slewX = new SlewRateLimiter(2);

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //Register named commads for pathplanner 
    NamedCommands.registerCommand("AimAtSpeaker", new AimAtSpeaker(m_VisionSubsystem, m_DriveSubsystem));
    NamedCommands.registerCommand("ShootAuto", new ShootAuto(m_ShooterSubsystem));
    NamedCommands.registerCommand("AutoPivot", new AutoPivot(m_PivotSubsystem, 220));
    NamedCommands.registerCommand("AutoPivotSide", new AutoPivot(m_PivotSubsystem, 210));
    //NamedCommands.registerCommand("PickUpInit", new ParallelCommandGroup(null)); //Command group to set the robot up to pick up rings during auton

    m_chooser = AutoBuilder.buildAutoChooser();
    field = new Field2d();
    
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

    // Configure the trigger bindings
    configureBindings();

    m_DriveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_DriveSubsystem.drive(
          -0.5*slewY.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftYAxis), 0.05)) ,
          -0.5*slewX.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftXAxis), 0.05)) ,
          -0.5*(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kRightXAxis), 0.05)),
          true, true), m_DriveSubsystem));

    m_ShooterSubsystem.setDefaultCommand(new RunCommand( () -> m_ShooterSubsystem.runShooter(0,0), m_ShooterSubsystem));

    // m_PivotSubsystem.setDefaultCommand(new PivotToAngle(m_PivotSubsystem, 270));

    m_Climb.setDefaultCommand(new RunCommand( () -> m_Climb.runClimb(0), m_Climb));
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
    
    //Button Onjects to asign bindings
    Trigger aButton1 = m_driverController1.button(ControlConstants.kAButton);
    Trigger xButton1 = m_driverController1.button(ControlConstants.kXButton);
    Trigger rBumper1 = m_driverController1.button(ControlConstants.kRightBumper);
    Trigger lBumper1 = m_driverController1.button(ControlConstants.kLeftBumper);
    Trigger bButton1 = m_driverController1.button(ControlConstants.kBButton);

    Trigger yButton1 = m_driverController1.button(ControlConstants.kYButton);

    Trigger bButton2 = m_driverController2.button(ControlConstants.kBButton);
    Trigger xButton2 = m_driverController2.button(ControlConstants.kXButton);
    Trigger aButton2 = m_driverController2.button(ControlConstants.kAButton);
    Trigger yButton2 = m_driverController2.button(ControlConstants.kYButton);
    Trigger lBumper2 = m_driverController2.button(ControlConstants.kLeftBumper);
    Trigger rBumper2 = m_driverController2.button(ControlConstants.kRightBumper);
    Trigger dPadUp2 = m_driverController2.povUp();
    Trigger dPadDown2 = m_driverController2.povDown();
    Trigger dPadLeft2 = m_driverController2.povLeft();
    Trigger dPadRight2 = m_driverController2.povRight();


    //Curremt range: ~200-245
    //Near Vertical 
    dPadUp2.onTrue(new PivotToAngle(m_PivotSubsystem, 195));
    
    //Side or 1 robot away
    dPadRight2.onTrue(new PivotToAngle(m_PivotSubsystem, 217));
    //Loading position
    dPadLeft2.onTrue(new PivotToAngle(m_PivotSubsystem, 240));
    //Shoot Straight Subwoofer
    dPadDown2.onTrue(new PivotToAngle(m_PivotSubsystem, 220));


    //Shooting from side subwoofer
    bButton2.onTrue(new PivotToAngle(m_PivotSubsystem, 210));
    //Shooting Trap
    aButton2.onTrue(new PivotToAngle(m_PivotSubsystem, 235));
    //xButton2.onTrue(new PivotToAngle(m_PivotSubsystem, pivotTune-=5));
    yButton2.whileTrue(new RunCommand( () -> m_Climb.runClimb(1)));
    xButton2.whileTrue(new RunCommand( () -> m_Climb.runClimb(-1)));


    bButton1.whileTrue(new TagShift(m_VisionSubsystem, m_DriveSubsystem));

    xButton1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.setX(), 
        m_DriveSubsystem));


    yButton1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.resetDrive(),
        m_DriveSubsystem));

    aButton1.onTrue(new AimAtSpeaker(m_VisionSubsystem, m_DriveSubsystem));

    rBumper2.whileTrue(new Shoot(m_ShooterSubsystem));

    lBumper2.onTrue(new LoadIntake(m_ShooterSubsystem));

    rBumper1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.drive( 
          -0.85*slewY.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftYAxis), 0.05)) ,
          -0.85*slewX.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftXAxis), 0.05)) ,
          -0.85*(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kRightXAxis), 0.05)),
          true, true), m_DriveSubsystem));  

    lBumper1.whileTrue(
      new RunCommand(
        () -> m_DriveSubsystem.drive( 
          -0.35*slewY.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftYAxis), 0.05)) ,
          -0.35*slewX.calculate(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kLeftXAxis), 0.05)) ,
          -0.35*(MathUtil.applyDeadband(m_driverController1.getRawAxis(Constants.ControlConstants.kRightXAxis), 0.05)),
          true, true), m_DriveSubsystem));  
      
    //xButton1.onTrue(
      //Reserved for climb spring release
    //  );
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
