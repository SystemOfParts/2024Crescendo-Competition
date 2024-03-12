package frc.robot.commands;
import java.util.Objects;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
public class LeadScrewForwardCommand extends SequentialCommandGroup{
    public LeadScrewForwardCommand(ArmSubsystem m_leadscrew){
        if (Objects.nonNull(m_leadscrew)){
            addCommands(
                //new InstantCommand(() -> m_leadscrew.leadScrewForward())
                );
        }
    }
}

