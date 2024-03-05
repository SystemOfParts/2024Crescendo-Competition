// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosBlue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.MoveToOrientationCommand;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoOneNote;
import frc.robot.commands.CheckToShoot;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueCenterThreeNote extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public AutoBlueCenterThreeNote(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      // Turn on the shooter, orient to SUBWOOFER, check that shooter is at speed, feed intake to shoot, wait .5 seconds
      new AutoOneNote(m_arm, m_shooter, m_intake),
      
      // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
        // this trajectory was modified slightly to move through the note to intake it
        new RunTrajectorySequenceRobotAtStartPoint("BlueCenterThreeNotePart1")
      ),

<<<<<<< HEAD:src/main/java/frc/robot/commands/AutosBlue/AutoBlueCenterThreeNote.java
      // with the shooter running, the intake off, and a note loaded, orient arm to the intake position AND starting to move to pick up the 3rd note
      // but STOP before just before we get to the 3rd note so we can shoot the 2nd note
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_PODIUM),
        // this trajectory was modified slightly to stop in front of the note w/ room for the intake
        new RunTrajectorySequenceRobotAtStartPoint("BlueCenterThreeNotePart2")
      ),
=======
      // After moving, wait .5 seconds to pretend to pick up a note, REPLACE THIS WITH NOTE DETECTION

      // with the shooter running, the intake off, and a note loaded, orient arm to the AUTO_PODIUM position 
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_PODIUM),
>>>>>>> 1ed0008aad39195e5dd3518041450f2f24bb11bc:src/main/java/frc/robot/commands/AutoThreeNoteCenter.java
      
      // Make sure the shooter is still at speed
      new CheckToShoot(m_shooter, m_intake),
      

      // Feed the intake to actually shoot (still using Podium speed and orientation)
      new InstantCommand(() -> m_intake.runIntake(true)),

      // move the arm down to intake position
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
      
      // path the robot backwards through the 3rd note to pick it up with the intake
<<<<<<< HEAD:src/main/java/frc/robot/commands/AutosBlue/AutoBlueCenterThreeNote.java
      new RunTrajectorySequenceRobotAtStartPoint("BlueCenterThreeNotePart3"),
=======
      new RunTrajectorySequenceRobotAtStartPoint("5142_ThreeNoteCenterPart2"),



      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_FAR_SHOT),


      new WaitCommand(.5),

      // Make sure the shooter is still at speed
      new CheckToShoot(m_shooter, m_intake),
      

      // Feed the intake to actually shoot (still using Podium speed and orientation)
      new InstantCommand(() -> m_intake.runIntake(true)),

      // TEMPORARY Wait .5 seconds for the shot to be fired
      new WaitCommand(.5),

      // At the end of auto use the TELEOP orientation command to move to TRAVEL to turn off shooter and intake
      new MoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL)
      
      /*// TEMPORARY After moving, wait .5 seconds to pretend to pick up a note, REPLACE THIS WITH NOTE DETECTION
      new WaitCommand(.5), 

      // TEMPORARY Stop the intake so we don't shoot it - this might just need to always be a part of the intake subsystem
      new IntakeStopCommand(m_intake),
>>>>>>> 1ed0008aad39195e5dd3518041450f2f24bb11bc:src/main/java/frc/robot/commands/AutoThreeNoteCenter.java

      // move back to PODIUM orientation w/ shooter 
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.PODIUM),

      // make sure the shooter is up to speed
      new CheckToShoot(m_shooter, m_intake),

      // Feed the intake to actually shoot (still using Podium speed and orientation)
      new InstantCommand(() -> m_intake.runIntake(true)));
      
      // END AUTO
<<<<<<< HEAD:src/main/java/frc/robot/commands/AutosBlue/AutoBlueCenterThreeNote.java
=======
      */
    );
>>>>>>> 1ed0008aad39195e5dd3518041450f2f24bb11bc:src/main/java/frc/robot/commands/AutoThreeNoteCenter.java
  }
}
