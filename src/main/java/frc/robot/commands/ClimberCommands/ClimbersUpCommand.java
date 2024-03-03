package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
public class ClimbersUpCommand extends ParallelCommandGroup{
    public ClimbersUpCommand(
        ClimberSubsystem m_climber

    ){

    addCommands(
        new RunCommand(() -> m_climber.climber2Up(.5)),
        new RunCommand(() -> m_climber.climber1Up(.5)));
        
    }
}