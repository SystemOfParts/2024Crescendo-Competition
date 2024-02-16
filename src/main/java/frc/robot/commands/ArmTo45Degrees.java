package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
public class ArmTo45Degrees extends SequentialCommandGroup{
    public ArmTo45Degrees(
        ArmSubsystem m_arm

    ){
    addCommands(
        new RunCommand(() -> m_arm.armTo45Degrees()));
    }
}