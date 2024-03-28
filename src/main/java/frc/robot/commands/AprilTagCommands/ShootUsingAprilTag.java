// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.VisionHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PHTNVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootUsingAprilTag extends SequentialCommandGroup {
  /** Creates a new ShootUsingLL. */
  public ShootUsingAprilTag(ShooterSubsystem m_shooter,
  IntakeSubsystem m_intake, double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     addCommands(
       new SequentialCommandGroup(
        new PrintCommand("Shooting Distance : " + (RobotContainer.phtnVisionSubsystem.getShootingDistance() - 1.1)), 
       new DeferredCommand(

           () -> new AprilTagShootingSequence(m_shooter, m_intake, PHTNVisionSubsystem.distanceToShoot - 1.1), 
           Set.of())),

           new PrintCommand("No AT Visible")

    //       () -> RobotContainer.llVisionSubsystem.isApriltagVisible() && LimelightHelpers.isInRange(RobotContainer.llVisionSubsystem.getShootingDistance(), 0.0, 4.0)
    //   )
     );
  }
}