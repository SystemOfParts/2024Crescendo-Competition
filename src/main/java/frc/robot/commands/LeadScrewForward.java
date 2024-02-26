package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ArmSubsystem;
public class LeadScrewForward extends SequentialCommandGroup{
    public LeadScrewForward(
        ArmSubsystem m_arm

    ){
    addCommands(
        new RunCommand(() -> m_arm.leadScrewForward()));
 
    }
}

