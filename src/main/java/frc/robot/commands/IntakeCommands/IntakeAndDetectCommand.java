// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAndDetectCommand extends Command {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  /** Creates a new FeedShooterCommand. */
  public IntakeAndDetectCommand( 
    ShooterSubsystem shooter,
    IntakeSubsystem intake){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("-- || || [ INTAKE AND DETECT INIT  ] || || --");
    //m_intake.runIntake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("-- || || [ INTAKE AND DETECT EXECUTE  ] || || --");
    /* if (m_intake.isNoteInIntake() && m_intake.isIntaking){
      if ((!m_intake.isShooting)&&(!m_intake.isEjecting)){
        //System.out.println("************************************************ stopping intake  ***");
        m_intake.isIntaking = false;
        //new IntakeStopCommand(RobotContainer.intakeSubsystem, true);
        m_intake.stopIntake(false);
      }
    } */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        //System.out.println("-- || || [ FEED SHOOTER INTERUPTED ] || || --");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
