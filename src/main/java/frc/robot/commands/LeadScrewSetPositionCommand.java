package frc.robot.commands;
import java.util.Objects;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
public class LeadScrewSetPositionCommand extends SequentialCommandGroup{
    public LeadScrewSetPositionCommand(ArmSubsystem m_arm, int position){
        System.out.println("*** LeadScrewSetPosition called to position: "+position);   
        if (Objects.nonNull(m_arm)){
            System.out.println("*** m_leadscrew was not null");   
            addCommands(   
                new InstantCommand(() -> m_arm.leadScrewSetPosition(position)));
        }
    }
}
