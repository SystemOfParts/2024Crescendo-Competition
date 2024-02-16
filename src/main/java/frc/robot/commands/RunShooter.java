package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ShooterSubsystem;
public class RunShooter extends SequentialCommandGroup{
    public RunShooter(
        ShooterSubsystem m_shooter

    ){
    addCommands(
        new RunCommand(() -> m_shooter.runShooter()));
 
    }
}