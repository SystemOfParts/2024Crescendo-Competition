package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class ClimbersUp extends ParallelCommandGroup{
    public ClimbersUp(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new RunCommand(() -> m_climber.climber2Up(.5)),
        new RunCommand(() -> m_climber.climber1Up(.5)));
    }
}