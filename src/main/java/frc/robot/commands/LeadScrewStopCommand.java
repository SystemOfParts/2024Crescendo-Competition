package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LeadScrewSubsystem;

public class LeadScrewStopCommand extends SequentialCommandGroup{
    public LeadScrewStopCommand(LeadScrewSubsystem m_leadscrew){
        if (Objects.nonNull(m_leadscrew)){
            addCommands(
                new InstantCommand(() -> m_leadscrew.leadScrewStop()));
        }
    }
}

