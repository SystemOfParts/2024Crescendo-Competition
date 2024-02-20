package frc.robot.commands.LeadScrewCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.LeadScrewSubsystem;
public class LeadScrewStop extends SequentialCommandGroup{
    public LeadScrewStop(
        LeadScrewSubsystem m_leadscrew

    ){
    addCommands(
        new InstantCommand(() -> m_leadscrew.leadScrewStop()));
 
    }
}
