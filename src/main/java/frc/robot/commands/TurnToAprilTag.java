package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAprilTag extends SequentialCommandGroup{
  public TurnToAprilTag(double targetAngleDegrees, DriveSubsystem drive, boolean relative) 
  {
  if (RobotContainer.phtnVisionSubsystem.isApriltagVisible()){
    addCommands(
        new TurnToDegreeIMU(targetAngleDegrees, drive, relative));
 
    }
  }
}