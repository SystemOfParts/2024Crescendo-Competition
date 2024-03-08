// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class TurnToDegreeIMU extends PIDCommand {
    private final DriveSubsystem m_drive;
    private boolean m_relative;
    private double m_degree;

    static double kP = 2;
    static double kI = 0;
    static double kD = 0;

    /**
     * Create a new TurnToDegreeGyro command.
     *
     * @param distance The distance to drive (inches)
     */
    public TurnToDegreeIMU(double zAngle, DriveSubsystem drive, boolean relative) {

        super(new PIDController(kP, kI, kD),
        
                // Close loop on heading
                RobotContainer.imuSubsystem::getYaw,
                // Set reference to target
                zAngle,
                // Pipe output to turn robot
                output -> drive.turn(output));
    

        // Require the drive
        m_drive = drive;
        m_relative = relative;
        m_degree = zAngle;
        System.out.println("                                           ----->>> [  STARTING  ]: TurnToDegreeIMU: ANGLE: " + m_degree);
        addRequirements(m_drive);
        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the setpoint before it is considered as having reached the
        // reference
        getController()
                .setTolerance(2, 8); // figure these out, they are in degrees
       //}
    }

    @Override
    public void execute() {
        System.out.println("                                           ----->>> [  MOVING  ]: TurnToDegreeIMU: ANGLE: " + m_degree);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("                                           ----->>> [  INTERRUPTED  ]: TurnToDegreeIMU: [ ANGLE ]: " + m_degree+ " [ INTERRUPTED ]: "+interrupted);
        super.end(interrupted);
        // if the command finished properly
        if (!interrupted){
            // rumble the Xbox controller
            //new XboxRumbleCommand(0.25, .25, RumbleType.kLeftRumble);
        }
        m_drive.stopRobot();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

        super.initialize();
        SmartDashboard.putNumber("Desired turning deg", m_degree);

        // Only zero the heading if we are turning relative (e.g., turn 3 degrees from  
        // my current position). Do not zero it if turning absolute (e.g., turn TO 55       // we are in absolute?
        // degrees)
        if (m_relative) {
            System.out.println("YAW WAS: "+ RobotContainer.imuSubsystem.getYaw());
            RobotContainer.imuSubsystem.setYawForTrajectory(RobotContainer.imuSubsystem.getYaw());
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        RobotContainer.imuSubsystem.restoreYawAfterTrajectory();
        System.out.println("                                           ----->>> [  FINISHED  ]: TurnToDegreeIMU: [ ANGLE ]: " + m_degree);
        System.out.println("YAW RESET TO: "+ RobotContainer.imuSubsystem.getYaw());
        return getController().atSetpoint();
    }
}