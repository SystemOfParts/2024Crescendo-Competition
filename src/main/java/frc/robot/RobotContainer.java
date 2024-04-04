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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.PHTNVisionSubsystem;
import frc.robot.subsystems.NoteDetectionPHTNVisionSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import frc.robot.lib.GPMHelpers;

import frc.robot.commands.*;
import frc.robot.commands.AprilTagCommands.AprilTagPrint;
import frc.robot.commands.AprilTagCommands.DetectAprilTagCommand;
import frc.robot.commands.AprilTagCommands.ShootUsingAprilTag;
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
import frc.robot.commands.PathingCommands.RunTrajectorySequenceRobotAtStartPoint;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
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
  public static final LEDSubsystem LEDs = LEDSubsystem.getInstance();
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
  // FAST

  private final Command m_FASTCenterStayShoot = new FAST_AutoBlueStayShoot(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueNorthStayShoot = new FAST_AutoBlueNorthStayShoot(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueSouthStayShoot = new FAST_AutoBlueSouthStayShoot(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTCenterTwoPiece = new FAST_AutoBlueCenterTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTCenterThreePieceSouth = new FAST_AutoBlueCenterThreeNoteSouth(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTCenterThreePieceNorth = new FAST_AutoBlueCenterThreeNoteNorth(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTCenterFourPiece = new FAST_AutoBlueCenterFourNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueNorthShootLeave = new FAST_AutoBlueNorthShootLeave(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueNorthTwoPiece = new FAST_AutoBlueNorthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueNorthTwoPieceMid1 = new FAST_AutoBlueNorthTwoNoteMid1(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueNorthThreePieceMid1 = new FAST_AutoBlueNorthThreeNoteMid1(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueNorthThreePieceMid2 = new FAST_AutoBlueNorthThreeNoteMid2(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueSouthShootLeave = new FAST_AutoBlueSouthShootLeave(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueSouthShootTwoPiece = new FAST_AutoBlueSouthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueSouthShootTwoPieceMid4= new FAST_AutoBlueSouthTwoNoteMid4(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueSouthShootTwoPieceMid5= new FAST_AutoBlueSouthTwoNoteMid5(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTBlueSouthShootSweep= new FAST_AutoBlueSouthSweep(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedNorthShootLeave = new FAST_AutoRedNorthShootLeave(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedNorthStayShoot = new FAST_AutoRedNorthStayShoot(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedNorthTwoPiece = new FAST_AutoRedNorthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedNorthTwoPieceMid1 = new FAST_AutoRedNorthTwoNoteMid1(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedNorthThreePieceMid1 = new FAST_AutoRedNorthThreeNoteMid1(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedNorthThreePieceMid2 = new FAST_AutoRedNorthThreeNoteMid2(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedSouthStayShoot = new FAST_AutoRedSouthStayShoot(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedSouthShootLeave = new FAST_AutoRedSouthShootLeave(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedSouthShootSweep= new FAST_AutoRedSouthSweep(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedSouthTwoPiece = new FAST_AutoRedSouthTwoNote(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedSouthTwoPieceCenter5 = new FAST_AutoRedSouthTwoNoteCenter5(armSubsystem, intakeSubsystem, shooterSubsystem);
  private final Command m_FASTRedSouthTwoPieceCenter4 = new FAST_AutoRedSouthTwoNoteCenter4(armSubsystem, intakeSubsystem, shooterSubsystem);
  
  
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

    /* // SET THE DEFAULT COMMAND ON XBOX LEFT TRIGGER TO DETECT THE ANGLE OF AN APRIL TAG
    phtnVisionSubsystem.setDefaultCommand(
      new DetectAprilTagCommand(
        () -> getLeftTrigger()));

    // SET THE DEFAULT COMMAND ON XBOX RIGHT TRIGGER TO DETECT THE ANGLE OF A DETECTED NOTE WITH PHOTONVISION
    noteDetectionPhtnVisionSubsystem.setDefaultCommand(
      new DetectNoteCommand(
      //new OrientToNoteCommand(
        () -> getRightTrigger())); */

  // UNCOMMENT TO TEST TRAJECTORIES
  //  trajectoryCalibration();
    setIfAllianceRed();
  }

  // USED ONLY FOR TESTING AUTOS USING UNUSED KEYS
  public void trajectoryCalibration() {
    new Trigger(m_operator1Controller.button(2))
        .whileTrue(new AprilTagPrint(llVisionSubsystem))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    
    // new Trigger(m_operator1Controller.button(11))
    //     .whileTrue(new AutoTrapFromEitherAMP(armSubsystem, intakeSubsystem, shooterSubsystem))
    //     .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
  }

  private void configureAutos(){
    m_chooser.addOption("FAST 1 BLUE/RED CTR STAY", m_FASTCenterStayShoot);
    m_chooser.addOption("FAST 1 BLUE NORTH STAY", m_FASTBlueNorthStayShoot);
    m_chooser.addOption("FAST 1 BLUE SOUTH STAY", m_FASTBlueSouthStayShoot);
    m_chooser.addOption("FAST 1 RED NORTH STAY", m_FASTRedNorthStayShoot);
    m_chooser.addOption("FAST 1 RED SOUTH STAY", m_FASTRedSouthStayShoot);
    m_chooser.addOption("FAST CTR 2", m_FASTCenterTwoPiece);
    m_chooser.addOption("FAST CTR 3 TO SOUTH B", m_FASTCenterThreePieceSouth);
    m_chooser.addOption("FAST CTR 3 TO NORTH B", m_FASTCenterThreePieceNorth);
    m_chooser.addOption("FAST CTR 4", m_FASTCenterFourPiece);
    m_chooser.addOption("FAST BLUE NORTH LEAVE", m_FASTBlueNorthShootLeave);
    m_chooser.addOption("FAST BLUE NORTH 2", m_FASTBlueNorthTwoPiece);
    m_chooser.addOption("FAST BLUE NORTH 2 MID1", m_FASTBlueNorthTwoPieceMid1);
    m_chooser.addOption("FAST BLUE NORTH 3 MID1", m_FASTBlueNorthThreePieceMid1);
    m_chooser.addOption("FAST BLUE NORTH 3 MID2", m_FASTBlueNorthThreePieceMid2);
    m_chooser.addOption("FAST BLUE SOUTH LEAVE", m_FASTBlueSouthShootLeave);
    m_chooser.addOption("FAST BLUE SOUTH 2", m_FASTBlueSouthShootTwoPiece);
    m_chooser.addOption("FAST BLUE SOUTH 2 MID4", m_FASTBlueSouthShootTwoPieceMid4);
    m_chooser.addOption("FAST BLUE SOUTH 2 MID5", m_FASTBlueSouthShootTwoPieceMid5);
    m_chooser.addOption("FAST RED NORTH LEAVE", m_FASTRedNorthShootLeave);
    m_chooser.addOption("FAST RED NORTH 2", m_FASTRedNorthTwoPiece);
    m_chooser.addOption("FAST RED NORTH 2 MID1", m_FASTRedNorthTwoPieceMid1);
    m_chooser.addOption("FAST RED NORTH 3 MID1", m_FASTRedNorthThreePieceMid1);
    m_chooser.addOption("FAST RED NORTH 3 MID2", m_FASTRedNorthThreePieceMid2);
    m_chooser.addOption("FAST RED SOUTH LEAVE", m_FASTRedSouthShootLeave);
    m_chooser.addOption("FAST RED SOUTH 2", m_FASTRedSouthTwoPiece);   
    m_chooser.addOption("FAST RED SOUTH 2 MID4", m_FASTRedSouthTwoPieceCenter4);   
    m_chooser.addOption("FAST RED SOUTH 2 MID5", m_FASTRedSouthTwoPieceCenter5);   
    m_chooser.addOption("FAST BLUE SOUTH SWEEP", m_FASTBlueSouthShootSweep);
    m_chooser.addOption("FAST RED SOUTH SWEEP", m_FASTRedSouthShootSweep);
    SmartDashboard.putData(m_chooser);
  }

  public void setLEDPattern(BlinkinPattern pattern) {
    LEDs.setPattern(pattern);
  }
  public void setStartPattern() {
    //LEDs.setAllianceColorBreath();
  }
  public void setAutoPattern() {
    //LEDs.setAllianceColorShot();
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
    .onTrue(new InstantCommand(()->RobotContainer.climberSubsystem.ClimberModeTurnOn())) //turns on the climbers
    .onFalse(new InstantCommand(()->RobotContainer.climberSubsystem.ClimberModeTurnOff()));
    //MANUAL RESET MODE

  new Trigger (m_operator2Controller.button(11))
    .onTrue(new InstantCommand(()->RobotContainer.shooterSubsystem.HumModeTurnOff()))
    .onFalse(new InstantCommand(()->RobotContainer.shooterSubsystem.HumModeTurnOn())); // Stops humming for testing (annoying)
    //MANUAL RESET MODE

  new Trigger (m_operator1Controller.button(10))
    .onTrue(new InstantCommand(()->RobotContainer.climberSubsystem.manualModeTurnOn())) //allows you to reset climbers after a match when the relative encoders have been rezeroed
    .onTrue(new InstantCommand(()->RobotContainer.armSubsystem.manualModeTurnOn()))
    .onFalse(new InstantCommand(()->RobotContainer.climberSubsystem.manualModeTurnOff()))
    .onFalse(new InstantCommand(()->RobotContainer.armSubsystem.manualModeTurnOff()));


  // SHOOT USING LIMELIGHT POSITION TO APRILTAG
  new Trigger(m_operator1Controller.button(2))
      .onTrue(new ShootUsingAprilTag());

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
  
  // ORIENT TO THE STARTLINE POSITION
  new Trigger(m_operator2Controller.button(5)) 
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.STARTLINE));
  new Trigger(m_operator1Controller.button(11)) 
    .onTrue(new TurnToDegreeIMU( 
      34, 
        () -> getDriverXAxis(),
        () -> getDriverYAxis(),
        driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  new Trigger(m_operator2Controller.button(9)) 
    .onTrue(new TurnToDegreeIMU( 
      -34, 
        () -> getDriverXAxis(),
        () -> getDriverYAxis(),
        driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
 
  // DRIVER QUICK ANGLE BINDINGS
  
  // ROTATE THE BOT TO FACE RIGHT
  new JoystickButton(xboxController, 2)
    .onTrue(new TurnToDegreeIMU( 
      -90,
      () -> getDriverXAxis(),
      () -> getDriverYAxis(),
      driveSubsystem,  
      false))

    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  // ROTATE THE BOT TO FACE LEFT
  new JoystickButton(xboxController, 3)

    .onTrue(new TurnToDegreeIMU( 
      90, 
      () -> getDriverXAxis(),
      () -> getDriverYAxis(),
      driveSubsystem, 
      false))
        
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  new JoystickButton(xboxController, 4)
     .onTrue(new TurnToDegreeIMU( 
      0, 
        () -> getDriverXAxis(),
        () -> getDriverYAxis(),
        driveSubsystem, false))
    .onFalse( new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));

  new JoystickButton(xboxController, 1)
 .onTrue(new TurnToDegreeIMU( 
      180, 
        () -> getDriverXAxis(),
        () -> getDriverYAxis(),
        driveSubsystem, false))
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

  
}

  // GET THE AUTO COMMAND FROM THE CHOOSER
  public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
     System.out.println("***************  **************    RUNNING AUTO    ************ **************");
     System.out.println("*                                                                            *");
     System.out.println("               INTAKE NULL?          "+intakeSubsystem);
     System.out.println("              SHOOTER NULL?          "+shooterSubsystem);
     System.out.println("                  ARM NULL?          "+armSubsystem);
     System.out.println("*                                                                            *");
     System.out.println("***************  ************** ****************** ************ **************");
    
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