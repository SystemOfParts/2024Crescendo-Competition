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
import frc.robot.subsystems.PHTNVisionSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import frc.robot.commands.*;
import frc.robot.commands.AutoOneNote;
import frc.robot.commands.AutosBlue.*;
import frc.robot.commands.AutosRed.*;
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
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
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
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final PHTNVisionSubsystem phtnVisionSubsystem = new PHTNVisionSubsystem("AprilTagCamera");
  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  
  //Define Controllers
  public static Controller xboxController;
  
  //Define and instantiate CommandControllers
  //Both of these control the one button box
  private final CommandGenericHID m_operator1Controller = new CommandGenericHID(0);
  private final CommandGenericHID m_operator2Controller = new CommandGenericHID(1);

  // A Data Log Manager file handle
  public static StringLogEntry myStringLog;

  // ========================================
  // === Variables for the alliance color ===
  // ========================================
  // If Apriltag detection is in place, need to track joystick directions in reverse so
  // from the RED alliance side the forward is back, right is left, and IMU is + 180 degrees

  public static boolean isAlianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  //Define the SendableChooser for autos
  public final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure driver interface - binding joystick objects to port numbers
    
    configureDriverInterface();
    configureBindings();
    configureAutos();

      // Configure the trigger bindings
      
      driveSubsystem.setDefaultCommand(
                new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));


    trajectoryCalibration();
  }

  private void configureAutos(){

    final Command m_AutoShootOnly = new AutoOneNote(armSubsystem, shooterSubsystem, intakeSubsystem);
    final Command m_AutoRedCenterTwoNote = new AutoRedCenterTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueCenterTwoNote = new AutoBlueCenterTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoRedCenterThreeNote = new AutoRedCenterThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueCenterThreeNote = new AutoBlueCenterThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoRedCenterFourNote = new AutoRedCenterFourNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueCenterFourNote = new AutoBlueCenterFourNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoRedNorthTwoNote = new AutoRedNorthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueNorthTwoNote = new AutoBlueNorthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoRedNorthTwoNoteTRAP = new AutoRedNorthTwoNoteTRAP(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueNorthTwoNoteTRAP = new AutoBlueNorthTwoNoteTRAP(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoRedSouthThreeNoteMid4 = new AutoRedSouthThreeNoteMid4(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueSouthThreeNoteMid4 = new AutoBlueSouthThreeNoteMid4(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoRedSouthThreeNoteMid5 = new AutoRedSouthThreeNoteMid5(armSubsystem, intakeSubsystem, shooterSubsystem);
    final Command m_AutoBlueSouthThreeNoteMid5 = new AutoBlueSouthThreeNoteMid5(armSubsystem, intakeSubsystem, shooterSubsystem);
    /*
    if (isAlianceRed){
      m_chooser.setDefaultOption("Shoot 1, stay", m_AutoShootOnly);
      m_chooser.addOption("RED Center 2", m_AutoRedCenterTwoNote);
      m_chooser.addOption("RED Center 3", m_AutoRedCenterThreeNote);
      m_chooser.addOption("RED Center 4", m_AutoRedCenterFourNote);
      m_chooser.addOption("RED North 2", m_AutoRedNorthTwoNote);
      m_chooser.addOption("RED North Trap", m_AutoRedNorthTwoNoteTRAP);
      m_chooser.addOption("RED South 3 Mid4", m_AutoRedSouthThreeNoteMid4);
      m_chooser.addOption("RED South 3 Mid5", m_AutoRedSouthThreeNoteMid5);
    } else {
      m_chooser.setDefaultOption("Shoot 1, stay", m_AutoShootOnly);
      m_chooser.addOption("BLUE Center 2", m_AutoBlueCenterTwoNote);
      m_chooser.addOption("BLUE Center 3", m_AutoBlueCenterThreeNote);
      m_chooser.addOption("BLUE Center 4", m_AutoBlueCenterFourNote);
      m_chooser.addOption("BLUE North 2", m_AutoBlueNorthTwoNote);
      m_chooser.addOption("BLUE North Trap", m_AutoBlueNorthTwoNoteTRAP);
      m_chooser.addOption("BLUE South 3 Mid4", m_AutoBlueSouthThreeNoteMid4);
      m_chooser.addOption("BLUE South 3 Mid5", m_AutoBlueSouthThreeNoteMid5);
    }
     */
    m_chooser.setDefaultOption("Shoot 1, stay", m_AutoShootOnly);
    m_chooser.addOption("BLUE Center 2", m_AutoBlueCenterTwoNote);
    m_chooser.addOption("BLUE Center 3", m_AutoBlueCenterThreeNote);
    m_chooser.addOption("BLUE Center 4", m_AutoBlueCenterFourNote);
    m_chooser.addOption("BLUE North 2", m_AutoBlueNorthTwoNote);
    m_chooser.addOption("BLUE North Trap", m_AutoBlueNorthTwoNoteTRAP);
    m_chooser.addOption("BLUE South 3 Mid4", m_AutoBlueSouthThreeNoteMid4);
    m_chooser.addOption("BLUE South 3 Mid5", m_AutoBlueSouthThreeNoteMid5);
    m_chooser.addOption("RED Center 2", m_AutoRedCenterTwoNote);
    m_chooser.addOption("RED Center 3", m_AutoRedCenterThreeNote);
    m_chooser.addOption("RED Center 4", m_AutoRedCenterFourNote);
    m_chooser.addOption("RED North 2", m_AutoRedNorthTwoNote);
    m_chooser.addOption("RED North Trap", m_AutoRedNorthTwoNoteTRAP);
    m_chooser.addOption("RED South 3 Mid4", m_AutoRedSouthThreeNoteMid4);
    m_chooser.addOption("RED South 3 Mid5", m_AutoRedSouthThreeNoteMid5);
    SmartDashboard.putData(m_chooser);
  }

 //instantiate drive controllers
  private void configureDriverInterface() {
      xboxController = new Controller(ControllerDevice.XBOX_CONTROLLER);
  }

  private double getDriverXAxis() {
      return xboxController.getLeftStickY();
  }

private double getDriverYAxis() {
      return xboxController.getLeftStickX();
}

private double getDriverOmegaAxis() {
      return xboxController.getLeftStickOmega();
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
    


  new Trigger (m_operator1Controller.button(9))
  .onTrue(new InstantCommand(()->RobotContainer.climberSubsystem.ClimberModeTurnOn()))
  .onFalse(new InstantCommand(()->RobotContainer.climberSubsystem.ClimberModeTurnOff()));


 
    //Independent Climber Controls
    
  new Trigger(m_operator1Controller.button(6)) //button 6 = left climber up
    .whileTrue(new LeftClimberUpCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(7)) //button 7 = left climber down
    .whileTrue(new LeftClimberDownCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(4)) //button 11 = right climber up
    .whileTrue(new RightClimberUpCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(5)) //button 2 = right climber down
    .whileTrue(new RightClimberDownCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));



    //zero robot yaw (new forward) = button 10
    new Trigger(m_operator2Controller.button(10))
    .onTrue(new InstantCommand(()->RobotContainer.imuSubsystem.zeroYaw()));  

  //Orientation Bindings
  new Trigger(m_operator2Controller.button(6)) //button 6 = Subwoofer shooting
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.SUBWOOFER));

  new Trigger(m_operator2Controller.button(7)) //button 7 = Home Position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.HOME));
    
  new Trigger(m_operator2Controller.button(8)) //button 8 = Trap scoring
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAP_SCORE));

    

  new Trigger(m_operator2Controller.button(1)) // button 1 = Travel position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));

  new Trigger(m_operator2Controller.button(2)) // button 2 = intake position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.INTAKE));

  new Trigger(m_operator2Controller.button(3)) // button 3 = spit out note
     .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
    .onFalse(new IntakeStopCommand(intakeSubsystem));

  // Intake Bindings
  new Trigger(m_operator2Controller.button(4))  //button 4 - amp position
  .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.AMP));

  new Trigger(m_operator2Controller.button(5)) //button 5 = podium shooting
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.PODIUM));

  

  // DRIVER QUICK ANGLE BINDINGS
  // Turn exactly right
 new JoystickButton(xboxController, 2)
    .onTrue(new TurnToDegreeIMU( -90, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
  // Turn exactly left
  new JoystickButton(xboxController, 3)
    .onTrue(new TurnToDegreeIMU( 90, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  // Face exactly forward
  new JoystickButton(xboxController, 4)
    .onTrue(new TurnToDegreeIMU( 0, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  // Face exactly backward
  new JoystickButton(xboxController, 1)
    .onTrue(new TurnToDegreeIMU( 180, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
  
  // Feed the note from the intake to the shooter to shoot - uses the intake to move the note and waits for the shooter to be at speed
  // When finished stop the intake (will be replaced when we can do note detection w/ IR sensor)
  // When finished orient to TRAVEL position for safe movement
  new JoystickButton(xboxController, 6)
    .whileTrue(new FeedShooterCommand(intakeSubsystem)) //driver's right bumper intakes to shoot, and then goes to travel position on release
    .onFalse(new IntakeStopCommand(intakeSubsystem));
   // .onFalse(new MoveToOrientationCommand(armSubsystem, shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));

  // 
  new JoystickButton(xboxController, 5)
    .onTrue(
        new TurnToDegreeIMU((180 - (phtnVisionSubsystem.getAprilTagZAngle())), driveSubsystem, false)) 


    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
}

/**
* Bindings to test simple swerve trajectories done in PathPlanner
*/
// to use these make sure you comment out the other uses of buttons before!!!!

public void trajectoryCalibration() {
  /*
  new Trigger(m_operator2Controller.button(1))
      .onTrue(new RunTrajectorySequenceRobotAtStartPoint("5142_1MeterForward"))
      .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
 */
  new Trigger(m_operator2Controller.button(10))
      .whileTrue(new AutoBlueCenterTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem))
      .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
/*
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
    return m_chooser.getSelected(); //AutoBlueCenterThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  }

  // Aliiance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (! alliance.isPresent()) {
        System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
        isAlianceRed = alliance.get() == DriverStation.Alliance.Red;
        System.out.println("*** RED Alliance: "+isAlianceRed);
    }
  }
  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }
}