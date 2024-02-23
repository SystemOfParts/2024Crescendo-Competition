package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ShooterSubsystem;
public class RunShooter extends SequentialCommandGroup{
    public RunShooter(
        ShooterSubsystem m_shooter

    ){
    addCommands(
        new InstantCommand(() -> m_shooter.runShooter()));
 
    }
}