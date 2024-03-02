// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.Devices.Controller;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LeadScrewSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import frc.robot.commands.*;
import frc.robot.commands.ClimberCommands.ClimbersDownCommand;
import frc.robot.commands.ClimberCommands.ClimbersStopCommand;
import frc.robot.commands.ClimberCommands.ClimbersUpCommand;
import frc.robot.commands.ClimberCommands.LeftBrakeOnCommand;
import frc.robot.commands.ClimberCommands.LeftClimberDownCommand;
import frc.robot.commands.ClimberCommands.LeftClimberStopCommand;
import frc.robot.commands.ClimberCommands.LeftClimberUpCommand;
import frc.robot.commands.ClimberCommands.RightBrakeOnCommand;
import frc.robot.commands.ClimberCommands.RightClimberDownCommand;
import frc.robot.commands.ClimberCommands.RightClimberStopCommand;
import frc.robot.commands.ClimberCommands.RightClimberUpCommand;
import frc.robot.commands.IntakeCommands.FeedShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  
  //Instantiate Subsystems
  public static final IMUSubsystem imuSubsystem = new IMUSubsystem();
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();


  //public static final LeadScrewSubsystem leadScrewSubsystem = new LeadScrewSubsystem();
  
  //Define Controllers
  public static Controller xboxController;
  
  //Define and instantiate CommandControllers
  //Both of these control the one button box
  private final CommandGenericHID m_operator1Controller = new CommandGenericHID(0);
  private final CommandGenericHID m_operator2Controller = new CommandGenericHID(1);
  

  //Define autos
  public static final String kDefaultAuto = "5142_1MeterForward";
  public static final String kCustomAuto  = "5142_1MeterRight";
  public static final String kCustomAuto2 = "5142_RotateLeft90and1Meter";
  public static final String kCustomAuto3 = "5142_Rotate180and1Meter";
  public static final String kCustomAuto4 = "5142_ComplexPath";
  public static final String kCustomAuto5 = "5142_TwoNotePart1";


  public String ChosenAuto;

  //Define the SendableChooser for autos
  public final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure driver interface - binding joystick objects to port numbers
    
    configureDriverInterface();
    configureBindings();

      // Configure the trigger bindings
      driveSubsystem.setDefaultCommand(
                new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

      // add autos to the chooser 
      m_chooser.setDefaultOption("1MeterForward", kDefaultAuto);
      m_chooser.addOption("1MeterRight", kCustomAuto);
      m_chooser.addOption("RotateLeft90and1Meter", kCustomAuto2);
      m_chooser.addOption("ComplexPath", kCustomAuto3);
      m_chooser.addOption("Rotate180and1Meter", kCustomAuto4);
      m_chooser.addOption("TwoNotePart1", kCustomAuto5);

     
      SmartDashboard.putData("Auto choices", m_chooser);
    
  }

 //instantiate drive controllers
  private void configureDriverInterface() {
      xboxController = new Controller(ControllerDevice.XBOX_CONTROLLER);
      //m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort)
       //System.out.println("Driver interface configured");
  }

  private double getDriverXAxis() {
      return xboxController.getLeftStickY();
      //return -m_driverController.getLeftX(),
  }

private double getDriverYAxis() {
      return xboxController.getLeftStickX();
      //return -m_driverController.getLeftY();
}

private double getDriverOmegaAxis() {
      return xboxController.getLeftStickOmega();
      //return -m_driverController.getRightX();
}

private boolean getDriverFieldCentric() {
        return true; //return !xboxController.Button(1); bumper
}
public double getRightTrigger() {
        return xboxController.getRawAxis(3);
}

public double getLeftTrigger() {
        return xboxController.getRawAxis(2);
}

public boolean getRightBumper() {
        return xboxController.getRawButton(6);

}

public boolean getYButton() {

  return xboxController.getRawButton(4);
}

public boolean getAButton() {

  return xboxController.getRawButton(1);
}





private void configureBindings() {

  //Dual Climber Bindings
    
  new Trigger(m_operator1Controller.button(4)) //button 4 = both climbers up , independent limiting
    .whileTrue(new ClimbersUpCommand(climberSubsystem))
    .onFalse(new ClimbersStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(5)) //button 5 = both climbers down , independent limiting
    .whileTrue(new ClimbersDownCommand(climberSubsystem))
    .onFalse(new ClimbersStopCommand(climberSubsystem));

    //Independent Climber Controls
    
  new Trigger(m_operator1Controller.button(6)) //button 6 = left climber up (limited)
    .whileTrue(new LeftClimberUpCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(7)) //button 7 = left climber down (limited)
    .whileTrue(new LeftClimberDownCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(11)) //button 11 = right climber up (limited)
    .whileTrue(new RightClimberUpCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(2)) //button 2 = right climber down (limited)
    .whileTrue(new RightClimberDownCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(8)) //button 8 = left brake
   .whileTrue(new LeftBrakeOnCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(3)) //button 3 = right brake
  .whileTrue(new RightBrakeOnCommand(climberSubsystem));

    //zero robot yaw (new forward) = button 10
  new Trigger(m_operator2Controller.button(10))
    .onTrue(new InstantCommand(()->RobotContainer.imuSubsystem.zeroYaw()));

  //Orientation Bindings

  new Trigger(m_operator2Controller.button(6)) //button 6 = Home position
      .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.HOME));

  new Trigger(m_operator2Controller.button(7)) //button 7 = Travel Position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));
    
  new Trigger(m_operator2Controller.button(8)) //button 8 = amp position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.AMP));


  new Trigger(m_operator2Controller.button(1)) // button 1 = intake position
         //.onTrue(new ArmDown(armSubsystem));
       .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.INTAKE));

  new Trigger(m_operator2Controller.button(2)) // button 2 = shooting (subwoofer) position
    //.onTrue(new ArmTo45Degrees(armSubsystem));
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.SUBWOOFER));

  new Trigger(m_operator2Controller.button(3)) // button 3 = far shooting position
    //.onTrue(new ArmUpPosition(armSubsystem));
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAP_SCORE));




  new Trigger(m_operator2Controller.button(4))  //button 4 - intake
    .onTrue(new InstantCommand(() -> intakeSubsystem.runIntake()))
    .onFalse(new IntakeStopCommand(intakeSubsystem));

  new Trigger(m_operator2Controller.button(5)) //button 5 = reverse intake
    .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
    .onFalse(new IntakeStopCommand(intakeSubsystem));
      


  new JoystickButton(xboxController, 6)
    .whileTrue(new FeedShooterCommand(intakeSubsystem, shooterSubsystem)) //driver's right bumper intakes to shoot, and then goes to travel position on release
    .onFalse(new IntakeStopCommand(intakeSubsystem))
    .onFalse(new MoveToOrientationCommand(armSubsystem, shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));

  
}
  
         
/*      
      //DO NOT UNCOMMENT UNTIL YOU UPDATE BUTTON IDs SO THEY DONT CONFLICT

       //remove to use controls
      //trajectoryCalibration(); //tag this out if using controls
      
  }

  /**
* Bindings to test simple swerve trajectories done in PathPlanner
*/
// to use these make sure you comment out the other uses of buttons before!!!!

public void trajectoryCalibration() {
  
  new Trigger(m_operator2Controller.button(1))
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("5142_1MeterForward"))
      .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
/* 
  new Trigger(m_operator2Controller.button(1))
      .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("5142_1MeterForward"))
      .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

  new Trigger(m_operator2Controller.button(2))
      .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("5142_1MeterRight"))
      .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

  new Trigger(m_operator2Controller.button(7))
      .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("5142_RotateLeft90and1Meter"))
      .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

  new Trigger(m_operator2Controller.button(8))
      .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("5142_Rotate180and1Meter"))
      .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    */
  
  /*new Trigger(m_operator2Controller.button(8))
      .whileTrue(new TurnToAngleZeroHeadingCommand(Rotation2d.fromDegrees(0)))
      .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
  new Trigger(m_operator2Controller.button(9))
      .whileTrue(new InstantCommand(RobotContainer.driveSubsystem::testOdometryUpdates));
      */
}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    ChosenAuto = m_chooser.getSelected();
    return new ThreeNoteAuto(armSubsystem, intakeSubsystem, shooterSubsystem);//RunTrajectorySequenceRobotAtStartPoint(ChosenAuto); //basic path testing
  }
}