package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
public class ArmTo90Degrees extends SequentialCommandGroup{
    public ArmTo90Degrees(
        ArmSubsystem m_arm

    ){
    addCommands(
        new RunCommand(() -> m_arm.armTo90Degrees()));
    }
}