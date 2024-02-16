package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ShooterSubsystem;
public class StopShooter extends SequentialCommandGroup{
    public StopShooter(
        ShooterSubsystem m_shooter

    ){
    addCommands(
        new InstantCommand(() -> m_shooter.stopShooter()));
 
    }
}