package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ShooterSubsystem;
public class StopShooterCommand extends SequentialCommandGroup{
    public StopShooterCommand(
        ShooterSubsystem m_shooter

    ){
    addCommands(
        new InstantCommand(() -> m_shooter.stopShooter()));
 
    }
}