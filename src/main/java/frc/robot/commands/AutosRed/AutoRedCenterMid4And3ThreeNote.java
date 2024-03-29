// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosRed;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonomousCommands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutonomousCommands.AutoShootFromDistance;
import frc.robot.commands.AutonomousCommands.AutoShootFromSubwoofer;
import frc.robot.commands.GPMCommands.CheckToShoot;
import frc.robot.commands.PathingCommands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRedCenterMid4And3ThreeNote extends SequentialCommandGroup {
  public AutoRedCenterMid4And3ThreeNote(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    addCommands(
      new InstantCommand(() -> RobotContainer.imuSubsystem.setYaw(180)),
      // Turn on the shooter, orient to SUBWOOFER, check that shooter is at speed, feed intake to shoot, wait .5 seconds
      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),

      // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
      new ParallelCommandGroup(
        //start moving WHILE we move to intake (after a 5 second pause)
        // begin moving to the next note
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterThreeToMid4AndPod"),
        new SequentialCommandGroup(
          // wait half a second then put the intake down
          new WaitCommand(.5),
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE)
        )    
      ),
      // could potentially speed this up by doing this WHILE moving
      // Turn on the shooter, orient to PODIUM, check that shooter is at speed, feed intake to shoot, wait .5 seconds
      // SHOOTING SEQUENCE PODIUM
      new AutoShootFromDistance(m_arm, m_shooter, m_intake),
      // SHOOTING SEQUENCE PODIUM COMPLETE

      new ParallelCommandGroup(
        //start moving WHILE we move to intake (after a 5 second pause)
        // begin moving to the next note
        new RunTrajectorySequenceRobotAtStartPoint("RedPodToMid3AndPod"),
        new SequentialCommandGroup(
          // wait half a second then put the intake down
          new WaitCommand(.5),
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE)
        )    
      ),
      
      // SHOOTING SEQUENCE PODIUM 
      new AutoShootFromDistance(m_arm, m_shooter, m_intake));
      // SHOOTING SEQUENCE PODIUM COMPLETE

      // END AUTO
  }
}
