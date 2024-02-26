package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ShooterSubsystem;
public class RunShooterCommand extends SequentialCommandGroup{
    public RunShooterCommand(
        ShooterSubsystem m_shooter

    ){
    addCommands(
        new RunCommand(() -> m_shooter.runShooter()));
 
    }
}