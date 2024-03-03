package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
public class LeftClimberDownCommand extends SequentialCommandGroup{
    public LeftClimberDownCommand(
        ClimberSubsystem m_climber

    ){
            addCommands(
        new RunCommand(() -> m_climber.climber1Down(.5)));
        
    }
}