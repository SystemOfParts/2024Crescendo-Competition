package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Objects;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.LeadScrewSubsystem;

public class LeadScrewBackward extends SequentialCommandGroup{
    public LeadScrewBackward(LeadScrewSubsystem m_leadscrew){
        if (Objects.nonNull(m_leadscrew)){
            addCommands(
                new RunCommand(() -> m_leadscrew.leadScrewBackward()));
        }
    }
}

