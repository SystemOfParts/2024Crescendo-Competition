// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooterCommand extends Command {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  /** Creates a new FeedShooterCommand. */
  public FeedShooterCommand( 
    ShooterSubsystem shooter,
    IntakeSubsystem intake){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("-- || || [ TRYING TO SHOOT - ARE WE AT SPEED  ????  ] || || --");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooterSubsystem.areShootersAtSpeed()) {
      //System.out.println("-- || || [   ''                                    ''   ] || || --");
      //System.out.println("-- || || [       SHOOTERS AT SPEED: SHOOOOOT!!!!!!      ] || || --");
      //System.out.println("-- || || [   ''                                    ''   ] || || --");
      RobotContainer.intakeSubsystem.runIntake(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        //System.out.println("-- || || [ FEED SHOOTER INTERUPTED ] || || --");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        //System.out.println("-- || || [ FEED SHOOTER FINISHED ] || || --");
    return false;
  }
}
