package frc.robot.commands;
import java.util.Objects;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LeadScrewSubsystem;

public class LeadScrewSetPosition extends SequentialCommandGroup{
    public LeadScrewSetPosition(LeadScrewSubsystem m_leadscrew, int position){
        System.out.println("*** LeadScrewForwardCommand called to position: "+position);   
        if (Objects.nonNull(m_leadscrew)){
            System.out.println("*** m_leadscrew was not null");   
            addCommands(   
                new InstantCommand(() -> m_leadscrew.leadScrewSetPosition(position)));
        }
    }
}
