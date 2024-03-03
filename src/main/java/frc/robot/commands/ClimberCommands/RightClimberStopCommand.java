package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
public class RightClimberStopCommand extends SequentialCommandGroup{
    public RightClimberStopCommand(
        ClimberSubsystem m_climber

    ){

    addCommands(
        new InstantCommand(() -> m_climber.stopClimber2()));
        
    }
}