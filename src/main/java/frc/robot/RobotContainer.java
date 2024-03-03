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
      // trajectoryCalibration();
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
    
  new Trigger(m_operator1Controller.button(4)) //button 4 = both climbers up 
    .whileTrue(new ClimbersUpCommand(climberSubsystem))
    .onFalse(new ClimbersStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(5)) //button 5 = both climbers down 
    .whileTrue(new ClimbersDownCommand(climberSubsystem))
    .onFalse(new ClimbersStopCommand(climberSubsystem));

    //Independent Climber Controls
    
  new Trigger(m_operator1Controller.button(6)) //button 6 = left climber up
    .whileTrue(new LeftClimberUpCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(7)) //button 7 = left climber down
    .whileTrue(new LeftClimberDownCommand(climberSubsystem))
    .onFalse(new LeftClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(11)) //button 11 = right climber up
    .whileTrue(new RightClimberUpCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(2)) //button 2 = right climber down
    .whileTrue(new RightClimberDownCommand(climberSubsystem))
    .onFalse(new RightClimberStopCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(8)) //button 8 = left brake
   .whileTrue(new LeftBrakeOnCommand(climberSubsystem));

  new Trigger(m_operator1Controller.button(3)) //button 3 = right brake
  .whileTrue(new RightBrakeOnCommand(climberSubsystem));

    //zero robot yaw (new forward) = button 10
  /* new Trigger(m_operator2Controller.button(10))
    .onTrue(new InstantCommand(()->RobotContainer.imuSubsystem.zeroYaw())); */

  //Orientation Bindings
  new Trigger(m_operator2Controller.button(6)) //button 6 = Home position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.HOME));

  new Trigger(m_operator2Controller.button(7)) //button 7 = Travel Position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));
    
  new Trigger(m_operator2Controller.button(8)) //button 8 = amp position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.AMP));

  new Trigger(m_operator2Controller.button(1)) // button 1 = intake position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.INTAKE));

  new Trigger(m_operator2Controller.button(2)) // button 2 = shooting (subwoofer) position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.SUBWOOFER));

  new Trigger(m_operator2Controller.button(3)) // button 3 = far shooting position
    .onTrue(new MoveToOrientationCommand(armSubsystem,  shooterSubsystem, intakeSubsystem, Orientations.TRAP_SCORE));

  // Intake Bindings
  new Trigger(m_operator2Controller.button(4))  //button 4 - intake
    .onTrue(new InstantCommand(() -> intakeSubsystem.runIntake()))
    .onFalse(new IntakeStopCommand(intakeSubsystem));

  new Trigger(m_operator2Controller.button(5)) //button 5 = reverse intake
    .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
    .onFalse(new IntakeStopCommand(intakeSubsystem));

  // DRIVER QUICK ANGLE BINDINGS
  // Turn exactly right
  new JoystickButton(xboxController, 20)
    .onTrue(new TurnToDegreeIMU( 90, driveSubsystem, false));

  // Turn exactly left
  new JoystickButton(xboxController, 21)
    .onTrue(new TurnToDegreeIMU( -90, driveSubsystem, false));

  // Face exactly forward
  new JoystickButton(xboxController, 22)
    .onTrue(new TurnToDegreeIMU( 0, driveSubsystem, false));

  // Face exactly backward
  new JoystickButton(xboxController, 23)
    .onTrue(new TurnToDegreeIMU( 180, driveSubsystem, false));
  
  // Feed the note from the intake to the shooter to shoot - uses the intake to move the note and waits for the shooter to be at speed
  // When finished stop the intake (will be replaced when we can do note detection w/ IR sensor)
  // When finished orient to TRAVEL position for safe movement
  new JoystickButton(xboxController, 6)
    .whileTrue(new FeedShooterCommand(intakeSubsystem)) //driver's right bumper intakes to shoot, and then goes to travel position on release
    .onFalse(new IntakeStopCommand(intakeSubsystem))
    .onFalse(new MoveToOrientationCommand(armSubsystem, shooterSubsystem, intakeSubsystem, Orientations.TRAVEL));

  // 
  new JoystickButton(xboxController, 5)
    .whileTrue(new TurnToDegreeIMU(phtnVisionSubsystem.getAprilTagZAngle(), driveSubsystem, getAButton()));
//add command cancel on false if needed
  
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
      .whileTrue(new AutoTwoNoteCenter(armSubsystem, intakeSubsystem, shooterSubsystem))
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
    ChosenAuto = m_chooser.getSelected();
    return new AutoThreeNoteCenter(armSubsystem, intakeSubsystem, shooterSubsystem);//RunTrajectorySequenceRobotAtStartPoint(ChosenAuto); //basic path testing
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