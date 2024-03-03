package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
public class RightBrakeOnCommand extends SequentialCommandGroup{
    public RightBrakeOnCommand(
        ClimberSubsystem m_climber

    ){
            
    addCommands(
        new RunCommand(() -> m_climber.rightBrakeOn()));
        
    }
}