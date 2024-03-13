// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 5;
	}

	public static final class CanIdConstants {
		public static final int kLeadScrewID = 13;
		public static final int kClimber1Id = 16;
		public static final int kClimber2Id = 17;
	}

	public static final class ClimberConstants {
		public static final double kSpeed = 0.25;
		public static final double kMAX_POSITION = -400.0;
  		public static final double kMIN_POSITION = -5;
	}

	public static final class IntakeConstants {
		public static final double kSpeed = .9;
		public static final double kEjectSpeed = -.75;
	}

	public static final class ShooterConstants {
        public static double kShooter1SetPoint = 4800;
        public static double kShooter2SetPoint = 3850;
    }

	public static final class OrientationConstants {
		// Arm Positions in order: arm, leadscrew, shooter on, intake on
		public enum Orientations {
			HOME("Home",
							-47,
							0,
							0,
							false,
							false,
							false,
							false),
			TRAVEL("Travel",
							-38.5,
							100,
							0,
							false,
							false,
							false,
							false),
			INTAKE("Intake",
							-48,
							580,
							0,
							false,
							true,
							true,
							false),
			AMP("Amp",
							0, //try -3 //90
							580,
							1200,
							true,
							false, 
							true,
							false),
			PODIUM("Podium",
							-34,//18
							580, 
							3000, 
							true,
							false,
							true,
							false),
			SUBWOOFER("Subwoofer",
							-38.5, //11
							580,
							1000,
							true,
							false,
							true,
							false),
			PRECLIMB("Preclimb",
							-49,
							400,
							0,
							false,
							false,
							false,
							false),
			AUTO_SUBWOOFER("AutoSubwoofer",
							-38.5, //11
							580,
							2500,
							true,
							false,
							false,
							false),
			AUTO_PODIUM("AutoPodium",
							-34, 
							580, 
							3000, 
							true,
							false,
							false,
							false),
			AUTO_INTAKE("AutoIntake",
							-48,
							580,
							2500,
							false,
							true,
							false,
							false),
			AUTO_FAR_SHOT("AutoFarShot",
							21,
							580,
							3000,
							true,
							false,
							false,
							false),
			TRAP_SCORE("Trap Score",
							-11,
							580,
							1450,
							true,
							false,
							true,
							false);

			public final String label;
			public final double armPosition;
			public final double leadScrewPosition;
			public final double shooterSpeed;
			public final boolean shooterOn;
			public final boolean intakeOn;
			public final boolean maintainHumSpeed;
			public final boolean leadScrewFirst;

			private Orientations(String label,
							double armPosition,
							double leadScrewPosition,
							double shooterSpeed,
							boolean shooterOn,
							boolean intakeOn,
							boolean maintainHumSpeed,
							boolean leadScrewFirst) {
					this.label = label;
					this.armPosition = armPosition;
					this.leadScrewPosition = leadScrewPosition;
					this.shooterSpeed = shooterSpeed;
					this.shooterOn = shooterOn;
					this.intakeOn = intakeOn;
					this.maintainHumSpeed = maintainHumSpeed;
					this.leadScrewFirst = leadScrewFirst;
			}

		}
    }
	
	
	/**
	 * This class contains configuration constants for the chassis, the individual
	 * swerve modules and the motors
	 * We try to distinguish between them by proper naming.
	 * It's likely we will need to do some additional cleanup here to make them easy
	 * to understand
	 */
	public static final class SwerveChassis {

		// TRAJECTORY PARAMETERS 3039
		/* Drive Feedforward */
		public static final double DRIVE_KS = 0.11937 / 12;
		public static final double DRIVE_KV = 2.6335 / 12;
		public static final double DRIVE_KA = 0.46034 / 12;

		/*
		 * Drive train properties
		 * All measurements are in meters
		 */
		public static final double TRACK_WIDTH = Units.inchesToMeters(21); // left to right
		public static final double WHEEL_BASE = Units.inchesToMeters(20.75); //front to back
		public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

		/*Swerve Auto Constants */

		// WATCH OUT: CHANGE DIMENSIONS

		public static final double CHASSIS_OUTER_DIMENSIONS_X = Units.inchesToMeters(26);	//meters
		public static final double CHASSIS_OUTER_DIMENSIONS_Y = Units.inchesToMeters(26);

		public static final double CHASSIS_OUTER_DRIVE_RADIUS = Math.sqrt( (CHASSIS_OUTER_DIMENSIONS_X * CHASSIS_OUTER_DIMENSIONS_X) + (CHASSIS_OUTER_DIMENSIONS_Y * CHASSIS_OUTER_DIMENSIONS_Y) );

		public static final double ANGLE_GEAR_RATIO = 1.0;
		public static final double DRIVE_GEAR_RATIO = 8.0;

		/**
		 * This class lists locating of each of the swerve modules from the center of
		 * the robot.
		 * It is assumed that there are four swerve modules located at the edges of a
		 * rectangle.
		 * It is also assumed that the center of rotation is at the center of that
		 * rectangle.
		 * 
		 * If your center of rotation is far off the center of that rectangle, which may
		 * happen
		 * due to the very scewed CG or uneven traction, you may want to adjust the
		 * numbers below
		 * based on the center of rotation.
		 * The order of the wheels location definition must match the order of the
		 * swerve modules
		 * defined in the DriveSubsystem for the SwerveModule array.
		 */
		public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

		/*
		 * Ramp Rates and Current Limits. Assumed to be the same for all drivetrain
		 * motors of the
		 * same type/purpose.
		 */
		public static final double DRIVE_CLOSED_LOOP_RAMP = 0;
		public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
		public static final int ANGLE_MOTOR_SMART_CURRENT = 25;
		public static final double ANGLE_MOTOR_SECONDARY_LIMIT = 40;
		public static final int DRIVE_MOTOR_SMART_CURRENT = 40;
		public static final double DRIVE_MOTOR_SECONDARY_LIMIT = 60;

		/**
		 * Angle Motor PID. Assumed to be the same for all angle motors
		 * These PID constants are only used for auto trajectory driving, and not
		 * teleop.
		 * Changes to these constants will have a substantial impact on the precision of
		 * your
		 * trajectory if it includes holonomic rotation.
		 * Make sure to test the values and adjust them as needed for your robot.
		 */
		public static final double ANGLE_CHASSIS_KP = 3.0; // 6.25
		public static final double ANGLE_CHASSIS_KI = 0.0; // 0.04
		public static final double ANGLE_CHASSIS_KD = 0.0; // 0.07

		public static final double ANGLE_MOTOR_MIN_OUTPUT = -1;
		public static final double ANGLE_MOTOR_MAX_OUTPUT = 1;

		public static final double ANGLE_MOTOR_PID_TIMEOUT = 30; // milliseconds

		public static final double ANGLE_MOTOR_VELOCITY_CONVERSION = 360.0 / 2048.0; // conversion factor from
																								// tick/100ms to
																								// degree/s

		/**
		 * Drive Motor PID. Assumed to be the same for all drive motors
		 * These PID constants are only used for auto trajectory driving, and not
		 * teleop.
		 * We found that changing them a bit will not have a substantial impact on the
		 * trajectory with PathPlanner
		 * even if a trajectory includes a holonomic component.
		 */
		public static final double DRIVE_CHASSIS_KP = 3.0;
		public static final double DRIVE_CHASSIS_KI = 0.05;
		public static final double DRIVE_CHASSIS_KD = 0.01;

		/**
		 * Maximum linear speed of chassis in meters per second
		 * Note that not determining this number precisely up front will not affect your
		 * teleop driving
		 * as the teleop logic will simply use it as a point of reference.
		 * Changing this number will not require any other changes in the teleop code.
		 */
		public static final double MAX_VELOCITY = 3.0;

		/**
		 * Radians per second.
		 * Swerve chassis assumes that the maximum linear speed during rotation is the
		 * same as the
		 * maxiumum linear speed during forward drive.
		 * That means the maximum angular speed can be calculated by dividing the
		 * maximum linear speed by
		 * the radius of rotation, which can be calculated by halving the distance
		 * between the opposing swerve
		 * modules such as the front left and rear right.
		 */
		public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY
				/ (Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2);

		// For trajectory driving.
		public static final double MAX_ACCELERATION = .9;

		/**
		 * Parameters for BaseMotorTalonSRX class
		 * This class is specific to the motors controlled by TalonSRX controller.
		 * Parameters specified here are primarily used in the motor configuration
		 * methods, as well as
		 * getters that translate encoder outputs from encoder-specific units to the SI
		 * units.
		 * Other motor controller implementations will likely have a different set of
		 * constants.
		 */

		public static final class NEOSwerveConfiguration {
			public static final double metersPerTick = 1.1931809; // TODO: measure this number on the robot
			//public static final double ticksPerFullRotation = 2048.0; 
			//public static final double degreePerTick = 360.0 / ticksPerFullRotation; // BOTH are copied over from TalonSRX
			public static final double ticksPerFullRotation = 2 * Math.PI;
			public static final double degreePerTick = 360/ticksPerFullRotation;

			public static final double nominalVoltage = 12.0;

			public static final int angleMotorCurrentLimit = 20;
			public static final int driveMotorCurrentLimit = 40;

			public static final double rampRate = 0.25;

			public static final double minInput = 0;
			public static final double maxInput = 2*Math.PI;

			public static final double positionConversionFactor = 0;

			public static final double DRIVE_PULSE_PER_ROTATION = 42;

			public static final double ANGLE_PULSE_PER_ROTATION = 0;


			//TODO: find that value pulse per rot

			

		}

		/**
		 * The following constants are used to print swerve telemetry. Please, note that
		 * excessive telemetry
		 * will cause excessive CPU usage, and ultimately will result in a packet loss.
		 * If after enabling telemetry you will see CPU approaching 100% in the RIO log,
		 * such settings will not be
		 * usable for extensive driving or competition. These settings were put in place
		 * so a team can
		 * troubleshoot teleop and trajectory driving.
		 */
		public static final class SwerveTelemetry {
			public static enum SwerveDriveOrTelemetry {
				DRIVE_ONLY,
				TELEMETRY_ONLY,
				DRIVE_AND_TELEMETRY;
			}

			/**
			 * Specify whether telemetry will be printed and/or the robot will apply power
			 * to the motors
			 */
			public static final SwerveDriveOrTelemetry swerveDriveOrTelemetry = SwerveDriveOrTelemetry.DRIVE_AND_TELEMETRY;

			/**
			 * Print odometry telemetry every 20 milliseconds.
			 */
			public static final boolean odometryTelemetryPrint = true;

		}

		/*
		 * Add controller types for each supported motor controller including simulated
		 * ones
		 * Make sure to modify BaseMotorPassthrough.java and add specific implementation
		 * class
		 * under the "Motor" folder
		 */
		public static enum BaseMotorControllerTypes {
			TALON_SRX,
			SPARKMAX;
		}

		/**
		 * Swerve Module Constants for each module including
		 * driveMotorType - type of the motor controller (e.g. TalonSRX vs NEO vs
		 * simulations)
		 * angleMotorType - type of the motor controller (e.g. TalonSRX vs NEO vs
		 * simulations)
		 * driveMotorID - CAN ID of the drive motors
		 * angleMotorID - CAN ID of the rotation motors
		 * angleOffset - Angle deviation of the absolute encoder when the
		 * respective wheel is pointing forward based on the absolute encoder value
		 * driveMotorInverted - is the drive motor inverted
		 * angleMotorInverted - is the angle motor inverted
		 * driveMotorSensorPhaseInverted - is the drive motor sensor phase inverted
		 * angleMotorSensorPhaseInverted - is the angle motor sensor phase inverted
		 * 
		 * For sensor phase we should use PID rule - when the positive power is applied,
		 * the motor should propell the robot "forward" and the corresponding encoder
		 * value should increase. Also for the angle motors the "positive" direction
		 * is counterclockwise.
		 * 
		 * Only include constants that may differ for each motor.
		 * Items that are the same for each motoro or motor type (e.g. PID constants)
		 * should be defined elsewhere.
		 * 
		 * Since this is an ENUM, need to have getter method for each value.
		 */
		public static enum SwerveModuleConstants {
			MOD0( // Front Left
					BaseMotorControllerTypes.SPARKMAX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					2, // driveMotorID
					1, // angleMotorID
					(5.675586 - Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor
			),
			MOD1( // Front Right
					BaseMotorControllerTypes.SPARKMAX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					4, // driveMotorID
					3, // angleMotorID
					(0.154961 + Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor

			),
			MOD2( // Back Left
					BaseMotorControllerTypes.SPARKMAX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					6, // driveMotorID
					5, // angleMotorID
					(6.141332 - Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor

			),
			MOD3( // Back Right
					BaseMotorControllerTypes.SPARKMAX, // Drive motor type
					BaseMotorControllerTypes.SPARKMAX, // Angle motor type
					8, // driveMotorID
					7, // angleMotorID
					(5.424344 - Math.PI) *360.0/NEOSwerveConfiguration.ticksPerFullRotation, // angleOffset
					false, // Inversion for drive motor
					true, // Inversion for angle motor
					false, // Sensor phase for drive motor
					true // Sensor phase for angle motor

			);

			private BaseMotorControllerTypes driveBaseMotorControllerType;
			private BaseMotorControllerTypes angleBaseMotorControllerType;
			private int driveMotorID;
			private int angleMotorID;
			private double angleOffset;
			private boolean driveMotorInverted;
			private boolean angleMotorInverted;
			private boolean driveMotorSensorPhase;
			private boolean angleMotorSensorPhase;

			SwerveModuleConstants(BaseMotorControllerTypes dm, BaseMotorControllerTypes am, int d, int a, double o,
					boolean di, boolean ai, boolean ds, boolean as) {
				this.driveBaseMotorControllerType = dm;
				this.angleBaseMotorControllerType = am;
				this.driveMotorID = d;
				this.angleMotorID = a;
				this.angleOffset = o;
				this.driveMotorInverted = di;
				this.angleMotorInverted = ai;
				this.driveMotorSensorPhase = ds;
				this.angleMotorSensorPhase = as;
			}

			public BaseMotorControllerTypes getDriveMotorControllerType() {
				return driveBaseMotorControllerType;
			}

			public BaseMotorControllerTypes getAngleMotorControllerType() {
				return angleBaseMotorControllerType;
			}

			public int getDriveMotorID() {
				return driveMotorID;
			}

			public int getAngleMotorID() {
				return angleMotorID;
			}

			public double getAngleOffset() {
				return angleOffset;
			}

			public boolean isDriveMotorInverted() {
				return driveMotorInverted;
			}

			public boolean isAngleMotorInverted() {
				return angleMotorInverted;
			}

			public boolean getDriveMotorSensorPhase() {
				return driveMotorSensorPhase;
			}

			public boolean getAngleMotorSensorPhase() {
				return angleMotorSensorPhase;
			}

		} // End ENUM SwerveModuleConstants

	} // End Swerve

	public static final class IMUConstants {

		public static enum IMUTypes {
			NavX;
		}

		public static final IMUTypes imuType = IMUTypes.NavX;
	} // End IMUConstants

	/**
	 * These Hardware PID constants are used by the individual swerve modules, and
	 * are used only by turn motors.
	 * We do not currently use Hardware PID for manual or trajectory driving.
	 */
	public static final class PIDConstantsForSwerveModules {

		
		public static final class NEOAngle {

			public static final double kP = .4; //.4
			public static final double kI = 0;
			public static final double kD = 2; //Changed from .8 2/27 - Jordan
			public static final double kF = 0.0;
			public static final double kiz = 0; // I-zone
			public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
			public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
			public static final double DefaultAcceptableError = 5; // Sensor units
			public static final double Izone = 250;
			public static final double PeakOutput = 0.5; // Closed Loop peak output
			public static final double NeutralDeadband = 0.001;

			public static final double outputMin= -1;
			public static final double outputMax = 1;

		}

	}

	/**
	 * Controller-related constants.
	 * Here we define port numbers, axis, deadbands, button numbers and various
	 * ability flags, such as use of the cube driving
	 */
	public static final class OIConstants {

		public static final int driverControllerPort = 5;

		public static final int robotCentricButton = 2;

		public static enum ControllerDeviceType {
			LOGITECH,
			PS5,
			XBOX
		}

		public static enum ControllerDevice {
			DRIVESTICK(
					0, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			TURNSTICK( // Controls the rotation of the swervebot
					1, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y																				
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			XBOX_CONTROLLER(
					5, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.1, // deadband X for Xbox
					0.1, // deadband Y for Xbox      
					0.1, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false);

			private ControllerDeviceType controllerDeviceType;
			private int portNumber;
			private double deadbandX;
			private double deadbandY;
			private double deadbandOmega;
			private boolean cubeControllerLeftStick;
			private boolean cubeControllerRightStick;

			ControllerDevice(
					int pn,
					ControllerDeviceType cdt,
					double dx,
					double dy,
					double dm,
					boolean ccL,
					boolean ccR) {
				this.portNumber = pn;
				this.controllerDeviceType = cdt;
				this.deadbandX = dx;
				this.deadbandY = dy;
				this.deadbandOmega = dm;
				this.cubeControllerLeftStick = ccL;
				this.cubeControllerRightStick = ccR;
			}

			public ControllerDeviceType getControllerDeviceType() {
				return controllerDeviceType;
			}

			public int getPortNumber() {
				return portNumber;
			}

			public double getDeadbandX() {
				return deadbandX;
			}

			public double getDeadbandY() {
				return deadbandY;
			}

			public double getDeadbandOmega() {
				return deadbandOmega;
			}

			public boolean isCubeControllerLeftStick() {
				return cubeControllerLeftStick;
			}

			public boolean isCubeControllerRightStick() {
				return cubeControllerRightStick;
			}
		}
	}

	public static final class VisionConstants {

		// Poses of important game elements
		// Direction is - front of the robot faces the element
		
		public static final Pose2d redSpeakerPose = new Pose2d(8.308467, 1.442593, new Rotation2d(0)).relativeTo(LimeLightConstants.centerFieldPose) ;
		public static final Translation2d redSpeakerTranslation = redSpeakerPose.getTranslation();

		public static final Pose2d blueSpeakerPose = new Pose2d(-8.308467, 1.442593, new Rotation2d(Math.PI)).relativeTo(LimeLightConstants.centerFieldPose) ;
		public static final Translation2d blueSpeakerTranslation = blueSpeakerPose.getTranslation();

		public static final Pose2d redAmpPose = new Pose2d(6.429883, 4.098925, new Rotation2d(Math.PI/2)).relativeTo(LimeLightConstants.centerFieldPose) ;
		public static final Translation2d redAmpTranslation = redAmpPose.getTranslation();
		// Facign down
		public static final Pose2d blueAmpPose = new Pose2d(-6.429883, 4.098925, new Rotation2d(Math.PI/2)).relativeTo(LimeLightConstants.centerFieldPose) ;
		public static final Translation2d blueAmpTranslation = blueAmpPose.getTranslation();

		// Ideal shooting poses - all of them - back to the target, hence Math.PI rotation transform is added to all

		// WATCH OUT: FIX THIS 
		// Facing backwards
		//public static final Transform2d redSpeakerShootingTransform = new Transform2d(-1, 0, new Rotation2d(Math.PI));
		//public static final Pose2d redSpeakerShootingPose = redSpeakerPose.transformBy(redSpeakerShootingTransform);
		// Facing forward
		//public static final Transform2d blueSpeakerShootingTransform = new Transform2d(2, 0, new Rotation2d(Math.PI));
		//public static final Pose2d blueSpeakerShootingPose = blueSpeakerPose.transformBy(blueSpeakerShootingTransform);
		// Facign down
		//public static final Transform2d redAmpShootingTransform = new Transform2d(0, -1, new Rotation2d(Math.PI));
		//public static final Pose2d redAmpShootingPose = redAmpPose.transformBy(redAmpShootingTransform);
		// Facign down
		//public static final Transform2d blueAmpShootingTransform = new Transform2d(2, 0, new Rotation2d(Math.PI));
		//public static final Pose2d blueAmpShootingPose = blueAmpPose.transformBy(blueAmpShootingTransform);

		// All cameras, both LL and PhotonVision, must be properly calibrated for use
		// per procedures indicated by the vendors.
		// LL calibration involves special downloadable sheet with tags on it,
		// while PhotonVision is calibrated via checkerboard.
		// All calibration sheets must be printed to proper size as we try using built-in
		// field pose estimators

		public static final class LimeLightConstants {

			// If changing this value, do not forget to set it in LL
			public static final String LLAprilTagName = "limelight-at";	// Limelight that will track Apriltags; may decide to use multiple ones

			// NEW origin from the old origin point of view in the old coordiinate system
			public static final Pose2d centerFieldPose = new Pose2d(-8.308467, -4.098925, new Rotation2d(0));

		}
		public static final class PhotonVisionConstants {

			public static final boolean PV_PRESENT = true;
			public static final boolean NOTE_PV_PRESENT = true;
			public static final String PVCameraName = "AprilTagCamera";
			// Camera position from center of the chassis / floor (for Z) point of view; it's looking backwards
			public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.3048, 0.0, 0.3175), new Rotation3d(0,0,Math.PI));
			public static final Transform3d robotToNoteCam = new Transform3d(new Translation3d(0.2900, -1.9, 0.2000), new Rotation3d(0,0,Math.PI));

		}
	}

}
