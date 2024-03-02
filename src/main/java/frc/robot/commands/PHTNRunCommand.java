package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.PHTNVisionSubsystem;

public class PHTNRunCommand extends SequentialCommandGroup{
    public PHTNRunCommand(
        PHTNVisionSubsystem m_PHTN

    ){if (Objects.nonNull(m_PHTN)){
    addCommands(
        new RunCommand(() -> m_PHTN.getPHTNData()));
    }
    }
}