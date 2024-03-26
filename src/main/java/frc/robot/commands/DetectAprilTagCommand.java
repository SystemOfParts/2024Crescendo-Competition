package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PHTNVisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;

public class DetectAprilTagCommand extends Command{
  private final DoubleSupplier m_triggerValue;
  private PHTNVisionSubsystem phtn;
  private boolean recentDetection = false;

  public DetectAprilTagCommand(DoubleSupplier triggerValue) 
  {
    m_triggerValue = triggerValue;
    addRequirements(RobotContainer.phtnVisionSubsystem);
  }

  @Override
  public void initialize() {
        phtn = RobotContainer.phtnVisionSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.intakeSubsystem.isNoteInIntake()){
      if (m_triggerValue.getAsDouble() > .1){
        phtn.getPHTNData();
        System.out.println("::::::::::ATG YAW: "+phtn.getAprilTagYaw());
        System.out.println("::::::IS APRIL TAG VIS?: "+phtn.isApriltagVisible());
        if (phtn.isApriltagVisible()){
          //System.out.println("::::::::::ATG YAW: "+phtn.getAprilTagYaw());
          double visibleYaw = phtn.getAprilTagYaw();
          if ((visibleYaw>-3)&&(visibleYaw<3)){
            RobotContainer.LEDs.setPattern(BlinkinPattern.BLUE);
            RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 1);
            recentDetection = true;
          } else if ((visibleYaw > -8)&&(visibleYaw <-3)){
            // TOO FAR LEFT, TURN ON RIGHT RUMBLE
            //RobotContainer.LEDs.setPattern(BlinkinPattern.ORANGE);
            RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, .3);
            recentDetection = true;
          } else if ((visibleYaw < 8)&&(visibleYaw > 3)){
            // TOO FAR RIGHT, TURN ON LEFT RUMBLE
            //RobotContainer.LEDs.setPattern(BlinkinPattern.ORANGE);
            RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, .3);
            recentDetection = true;
          } else {
            // TOO FAR OFF - TURN OFF ALL RUMBLE
            //RobotContainer.LEDs.setPattern(BlinkinPattern.ORANGE);
            RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
            RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
            recentDetection = false;
          }
        }  else {
          // no April tag is visible so turn off rumble
          //RobotContainer.LEDs.setPattern(BlinkinPattern.ORANGE);
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0); 
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          recentDetection = false;
        }
      } else {
        if (recentDetection) {
          //RobotContainer.LEDs.setPattern(BlinkinPattern.ORANGE);
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0); 
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          recentDetection = false;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}