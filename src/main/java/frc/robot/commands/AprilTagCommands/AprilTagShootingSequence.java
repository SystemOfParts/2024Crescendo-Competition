// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.FeedShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeOnCommand;
import frc.robot.commands.IntakeCommands.ShootingPushNote;
import frc.robot.lib.GPMHelpers;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagShootingSequence extends SequentialCommandGroup {
  IntakeSubsystem m_intake;
  
  public AprilTagShootingSequence(double distance) {
    addCommands(
// Spin the shooter first
       new WaitCommand(1.5)
        .raceWith(
// get shooter up to speed
           new RunShooterBasedOnDistanceCommand(RobotContainer.gpmHelpers.getGPM0ShooterPower(distance))
             .alongWith(
               new MoveToAngleBasedOnDistanceCommand(RobotContainer.gpmHelpers.getGPM0Angle(distance))
             )
         ),
        new ConditionalCommand(

// push note to shooter after shooter rollers get up to speed
         new WaitCommand(0.75)
           .raceWith(
              new ShootingPushNote(distance))
// after the note leaves the intake, wait to make sure it leaves the shooter as well
          .andThen(new WaitCommand(0.2))
          ,
          // slower command if the note is not seen by the sensor at this point
          (new WaitCommand(1.5)
            .raceWith(new IntakeOnCommand(m_intake))
           ) ,
          // is note seen by the sensor?
          RobotContainer.intakeSubsystem::isNoteInIntake),
        // wait until the shooting is done
       new PrintCommand("AprilTagShootingSequence done")
    );
    
  }
}