package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ArmSubsystem;
public class LeadScrewStop extends SequentialCommandGroup{
    public LeadScrewStop(
        ArmSubsystem m_arm

    ){
    addCommands(
        new InstantCommand(() -> m_arm.leadScrewStop()));
 
    }
}

