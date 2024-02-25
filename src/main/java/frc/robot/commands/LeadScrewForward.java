package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.LeadScrewSubsystem;
public class LeadScrewForward extends SequentialCommandGroup{
    public LeadScrewForward(
        LeadScrewSubsystem m_leadscrew

    ){
    addCommands(
        new RunCommand(() -> m_leadscrew.leadScrewForward()));
 
    }
}
