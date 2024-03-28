// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.Devices.Controller;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LLVisionSubsystem;
//import frc.robot.subsystems.LeadScrewSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PHTNVisionSubsystem;
import frc.robot.subsystems.NoteDetectionPHTNVisionSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import frc.robot.lib.GPMHelpers;

import frc.robot.commands.*;
import frc.robot.commands.AprilTagCommands.DetectAprilTagCommand;
import frc.robot.commands.AutonomousCommands.AutoShootAndStay;
import frc.robot.commands.AutosBlue.*;
import frc.robot.commands.AutosNeutral.AutoEitherCenterTwoNote;
import frc.robot.commands.AutosNeutral.AutoPIDAcrossUpDown;
import frc.robot.commands.AutosNeutral.AutoPIDTuneHoloInside;
import frc.robot.commands.AutosNeutral.AutoPIDTuneHoloOutside;
import frc.robot.commands.AutosNeutral.AutoEitherCenterOneNoteLeave;
import frc.robot.commands.AutosNeutral.AutoTrapFromEitherSpeaker;
import frc.robot.commands.AutosNeutral.AutoTrapFromEitherAMP;
import frc.robot.commands.AutosRed.*;
import frc.robot.commands.ClimberCommands.LeftClimberDownCommand;
import frc.robot.commands.ClimberCommands.LeftClimberStopCommand;
import frc.robot.commands.ClimberCommands.LeftClimberUpCommand;
import frc.robot.commands.ClimberCommands.RightClimberDownCommand;
import frc.robot.commands.ClimberCommands.RightClimberStopCommand;
import frc.robot.commands.ClimberCommands.RightClimberUpCommand;
import frc.robot.commands.GPMCommands.MoveToOrientationCommand;
import frc.robot.commands.IntakeCommands.FeedShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeOnCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.commands.NoteDetectionCommands.DetectNoteCommand;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LEDSubsystem;

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
  //public static final LEDSubsystem LEDs = LEDSubsystem.getInstance();
  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final PHTNVisionSubsystem phtnVisionSubsystem = new PHTNVisionSubsystem("AprilTagCamera");
  public final static LLVisionSubsystem llVisionSubsystem = new LLVisionSubsystem();
  public static final NoteDetectionPHTNVisionSubsystem noteDetectionPhtnVisionSubsystem = new NoteDetectionPHTNVisionSubsystem("NoteCamera");
  public final static GPMHelpers gpmHelpers = new GPMHelpers();
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
  
  // Neutral
  private final Command m_AutoShootOnly = new AutoShootAndStay(armSubsystem, shooterSubsystem, intakeSubsystem);
  private final Command m_AutoEitherCenterOneAndLeave = new AutoEitherCenterOneNoteLeave(armSubsystem, shooterSubsystem, intakeSubsystem);
  private final Command m_AutoEitherCenterTwoNote = new AutoEitherCenterTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoTrapEitherSub = new AutoTrapFromEitherSpeaker(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoTrapEitherAMP = new AutoTrapFromEitherAMP(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_PIDAcrossUpDown = new AutoPIDAcrossUpDown(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_PIDHoloInside = new AutoPIDTuneHoloInside();
  private final Command m_PIDHoloOutside = new AutoPIDTuneHoloOutside();

  // Blue
  private final Command m_AutoBlueNorthOneAndLeave = new AutoBlueNorthOneNoteLeave(armSubsystem, shooterSubsystem, intakeSubsystem);
  private final Command m_AutoBlueNorthTwoNote = new AutoEitherCenterTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterFourNote = new AutoBlueCenterFourNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterThreeNoteNorth = new AutoBlueCenterThreeNoteNorth(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterThreeNoteSouth = new AutoBlueCenterThreeNoteSouth(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterMid2And3ThreeNote = new AutoBlueCenterMid2And3ThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterMid4And3ThreeNote = new AutoBlueCenterMid4And3ThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterMid2ToSubThreeNote = new AutoBlueCenterMid2ToSubThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterMid3ToSubThreeNote = new AutoBlueCenterMid3ToSubThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueCenterMid4ToSubThreeNote = new AutoBlueCenterMid4ToSubThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoBlueSouthOneAndLeave = new AutoBlueSouthOneNoteLeave(armSubsystem, shooterSubsystem, intakeSubsystem);
  private final Command m_AutoBlueSouthTwoNoteComplete = new AutoBlueSouthTwoNoteComplete(armSubsystem, intakeSubsystem, shooterSubsystem);
  
  // Red
  //private final Command m_AutoRedCenterFourNote = new AutoRedCenterFourNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedNorthTwoNote = new AutoRedNorthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterThreeNoteNorth = new AutoRedCenterThreeNoteNorth(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterThreeNoteSouth = new AutoRedCenterThreeNoteSouth(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterMid2And3ThreeNote = new AutoRedCenterMid2And3ThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterMid4And3ThreeNote = new AutoRedCenterMid4And3ThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterMid2ToSubThreeNote = new AutoRedCenterMid2ToSubThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterMid3ToSubThreeNote = new AutoRedCenterMid3ToSubThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedCenterMid4ToSubThreeNote = new AutoRedCenterMid4ToSubThreeNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_AutoRedSouthTwoNoteComplete = new AutoRedSouthTwoNoteComplete(armSubsystem, intakeSubsystem, shooterSubsystem);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure driver interface - binding joystick objects to port numbers
    
    configureDriverInterface();
    configureBindings();
    configureAutos();

    // SET THE DEFAULT COMMAND FOR DRIVING SWERVE
    driveSubsystem.setDefaultCommand(
      new DriveManuallyCommand(
        () -> getDriverXAxis(),
        () -> getDriverYAxis(),
        () -> getDriverOmegaAxis(),
        () -> getDriverFieldCentric()));

    // SET THE DEFAULT COMMAND ON XBOX LEFT TRIGGER TO DETECT THE ANGLE OF AN APRIL TAG
    phtnVisionSubsystem.setDefaultCommand(
      new DetectAprilTagCommand(
        () -> getLeftTrigger()));

    // SET THE DEFAULT COMMAND ON XBOX RIGHT TRIGGER TO DETECT THE ANGLE OF A DETECTED NOTE WITH PHOTONVISION
    noteDetectionPhtnVisionSubsystem.setDefaultCommand(
      new DetectNoteCommand(
      //new OrientToNoteCommand(
        () -> getRightTrigger()));

  // UNCOMMENT TO TEST TRAJECTORIES
    trajectoryCalibration();
    setIfAllianceRed();
  }

  // USED ONLY FOR TESTING AUTOS USING UNUSED KEYS
  public void trajectoryCalibration() {
    new Trigger(m_operator1Controller.button(2))
        .whileTrue(new AutoTrapFromEitherSpeaker(armSubsystem, intakeSubsystem, shooterSubsystem))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    
    new Trigger(m_operator1Controller.button(11))
        .whileTrue(new AutoTrapFromEitherAMP(armSubsystem, intakeSubsystem, shooterSubsystem))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
  }

  private void configureAutos(){
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
    m_chooser.setDefaultOption("BLUE/RED ANYWHERE Shoot 1, stay", m_AutoShootOnly);
    m_chooser.addOption("BLUE/RED Center 1 leave", m_AutoEitherCenterOneAndLeave);
    m_chooser.addOption("BLUE/RED Center 2", m_AutoEitherCenterTwoNote);
    m_chooser.addOption("BLUE North 1 leave", m_AutoBlueNorthOneAndLeave);
    m_chooser.addOption("BLUE North 2", m_AutoBlueNorthTwoNote);
    m_chooser.addOption("BLUE Center 3 North", m_AutoBlueCenterThreeNoteNorth);
    m_chooser.addOption("BLUE Center 3 South", m_AutoBlueCenterThreeNoteSouth);
    m_chooser.addOption("BLUE Center 3 - 2 AND 3", m_AutoBlueCenterMid2And3ThreeNote);
    m_chooser.addOption("BLUE Center 3 - 4 AND 3", m_AutoBlueCenterMid4And3ThreeNote);
    m_chooser.addOption("BLUE Center 3 - 2 AND Back", m_AutoBlueCenterMid2ToSubThreeNote);
    m_chooser.addOption("BLUE Center 3 - 3 AND Back", m_AutoBlueCenterMid3ToSubThreeNote);
    m_chooser.addOption("BLUE Center 3 - 4 AND Back", m_AutoBlueCenterMid4ToSubThreeNote);
    m_chooser.addOption("BLUE Center 4", m_AutoBlueCenterFourNote);
    m_chooser.addOption("BLUE South 1 leave", m_AutoBlueSouthOneAndLeave);
    m_chooser.addOption("BLUE South 2", m_AutoBlueSouthTwoNoteComplete);
    m_chooser.addOption("RED North 2", m_AutoRedNorthTwoNote);
    m_chooser.addOption("RED Center 3 South", m_AutoRedCenterThreeNoteSouth);
    m_chooser.addOption("RED Center 3 - 2 AND 3", m_AutoRedCenterMid2And3ThreeNote);
    m_chooser.addOption("RED Center 3 - 4 AND 3", m_AutoRedCenterMid4And3ThreeNote);
    m_chooser.addOption("RED Center 3 - 2 AND Back", m_AutoRedCenterMid2ToSubThreeNote);
    m_chooser.addOption("RED Center 3 - 3 AND Back", m_AutoRedCenterMid3ToSubThreeNote);
    m_chooser.addOption("RED Center 3 - 4 AND Back", m_AutoRedCenterMid4ToSubThreeNote);
    m_chooser.addOption("RED South 2", m_AutoRedSouthTwoNoteComplete);
    m_chooser.addOption("BLUE/RED TRAP from Center", m_AutoTrapEitherSub);
    m_chooser.addOption("BLUE/RED TRAP from AMP", m_AutoTrapEitherAMP);
    SmartDashboard.putData(m_chooser);
  }

 // INSTANTIATE XBOX DRIVER CONTROLLER
  private void configureDriverInterface() {
      xboxController = new Controller(ControllerDevice.XBOX_CONTROLLER);
  }

// RETURN XBOX LEFT JOYSTICK X AXIS
private double getDriverXAxis() {
    return xboxController.getLeftStickY();
}

// RETURN XBOX LEFT JOYSTICK Y AXIS
private double getDriverYAxis() {
      return xboxController.getLeftStickX();
}

// RETURN XBOX RIGHT JOYSTICK
private double getDriverOmegaAxis() {
      return xboxController.getLeftStickOmega();
}

// ALWAYS ORIENT TO FIELD CENTRIC
private boolean getDriverFieldCentric() {
        return true; 
}

// RETURN XBOX RIGHT TRIGGER
public static double getRightTrigger() {
        return xboxController.getRawAxis(3);
}

// RETURN XBOX LEFT TRIGGER
public static double getLeftTrigger() {
        return xboxController.getRawAxis(2);
}

// RETURN XBOX LEFT TRIGGER PRESSED AT ALL UNUSED?
public static boolean isLeftTriggerPressed(){
  return (getLeftTrigger() > .1);
}

// RETURN XBOX RIGHT TRIGGER PRESSED AT ALL UNUSED?
public static boolean isRightTriggerPressed(){
  return (getRightTrigger() > .1);
}

// RETURN XBOX RIGHT BUMPER
public boolean getRightBumper() {
        return xboxController.getRawButton(6);
}

// RETURN XBOX Y BUTTON
public boolean getYButton() {
  return xboxController.getRawButton(4);
}

// RETURN XBOX A BUTTON
public boolean getAButton() {
  return xboxController.getRawButton(1);
}

private void configureBindings() {    

  // ENABLE CLIMBING MODE
  new Trigger (m_operator1Controller.button(9))
    .onTrue(new InstantCommand(()->RobotContainer.climberSubsystem.ClimberModeTurnOn()))
    .onFalse(new InstantCommand(()->RobotContainer.climberSubsystem.ClimberModeTurnOff()));
    //MANUAL RESET MODE

      new Trigger (m_operator2Controller.button(12))
    .onTrue(new InstantCommand(()->RobotContainer.climberSubsystem.manualModeTurnOn()))
    .onTrue(new InstantCommand(()->RobotContainer.armSubsystem.manualModeTurnOn()))
    .onFalse(new InstantCommand(()->RobotContainer.climberSubsystem.manualModeTurnOff()))
    .onFalse(new InstantCommand(()->RobotContainer.armSubsystem.manualModeTurnOff()));
  
  // MOVE THE LEFT CLIMBER UP
  new Trigger(m_operator1Controller.button(6)) 
    .whileTrue(new LeftClimberUpCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  // MOVE THE LEFT CLIMBER DOWN  
  new Trigger(m_operator1Controller.button(7)) 
    .whileTrue(new LeftClimberDownCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  // MOVE TO THE PRECLIMB POSITION
  new Trigger(m_operator1Controller.button(8))
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.PRECLIMB));

  // MOVE THE RIGHT CLIMBER UP
  new Trigger(m_operator1Controller.button(4)) 
    .whileTrue(new RightClimberUpCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  // MOVE THE RIGHT CLIMBER DOWN
  new Trigger(m_operator1Controller.button(5)) 
    .whileTrue(new RightClimberDownCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  // SWITCH DRIVING TO BE FIELD ORIENTED
  new Trigger(m_operator2Controller.button(10))
    .onTrue(new InstantCommand(()->RobotContainer.imuSubsystem.zeroYaw()));  

  // ORIENT TO THE SUBWOOFER POSITION
  new Trigger(m_operator2Controller.button(6)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.SUBWOOFER));

  // ORIENT TO THE PODIUM POSITION
  new Trigger(m_operator2Controller.button(7)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.PODIUM));
  
  // ORIENT TO THE AMP POSITION
  new Trigger(m_operator2Controller.button(8)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.AMP));

  // ORIENT TO THE HOME POSITION
  new Trigger(m_operator2Controller.button(1)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.HOME));

  // ORIENT TO THE TRAVEL POSITION
  new Trigger(m_operator2Controller.button(2)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));

  // ORIENT TO THE TRAVEL POSITION
  new Trigger(m_operator1Controller.button(3)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.FEEDER));

  // REVERSE THE INTAKE TO SPIT OUT A NOTE
  new Trigger(m_operator2Controller.button(4))
    .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
    .onFalse(new IntakeStopCommand(intakeSubsystem, false));

  // ORIENT TO THE INTAKE POSITION
  new Trigger(m_operator2Controller.button(3))  
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.INTAKE));
  
  // ORIENT TO THE TRAP_SCORE POSITION
  new Trigger(m_operator2Controller.button(5)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.STARTLINE));

  // DRIVER QUICK ANGLE BINDINGS
  
  // ROTATE THE BOT TO FACE RIGHT
  new JoystickButton(xboxController, 2)
    .onTrue(new TurnToDegreeIMU( -90, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
  // ROTATE THE BOT TO FACE LEFT
  new JoystickButton(xboxController, 3)
    .onTrue(new TurnToDegreeIMU( 90, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  // ROTATE THE BOT TO ANGLE TOWARD THE STAGE RIGHT
  new JoystickButton(xboxController, 4)
    .onTrue(new TurnToDegreeIMU( -120, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  // ROTATE THE BOT TO ANGLE TOWARD THE STAGE LEFT
  new JoystickButton(xboxController, 1)
    .onTrue(new TurnToDegreeIMU( 120, driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
  
  // Feed the note from the intake to the shooter to shoot - uses the intake to move the note and waits for the shooter to be at speed
  // When finished orient to TRAVEL position for safe movement
  new JoystickButton(xboxController, 6)
    .whileTrue(new FeedShooterCommand(shooterSubsystem, intakeSubsystem)) //driver's right bumper intakes to shoot, and then goes to travel position on release
    .onFalse(new IntakeStopCommand(intakeSubsystem, false));

  /* 
  new JoystickButton(xboxController, 5)
    .onTrue(
        new TurnToDegreeIMU((180 - (phtnVisionSubsystem.getAprilTagZAngle())), driveSubsystem, false)) 


    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
  */    
}

  // GET THE AUTO COMMAND FROM THE CHOOSER
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
        //System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
        isAlianceRed = alliance.get() == DriverStation.Alliance.Red;
        //System.out.println("*** RED Alliance: "+isAlianceRed);
    }
  }
  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }
}