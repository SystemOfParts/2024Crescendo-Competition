package frc.robot.PassThroughSystems.Motor;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.PIDConstantsForSwerveModules.NEOAngle;
import frc.robot.Constants.SwerveChassis.NEOSwerveConfiguration;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstants;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

/*
 * This is a specific base motor implementation for the motors connected to SparkMAX NEOs
 */
public class BaseMotorNEO implements BaseMotorInterface {
    private CANSparkMax motorNEO;
    //private int CANID;
    private SwerveModuleConstants cAngle;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoderRel;
    private SparkAbsoluteEncoder angleEncoder;
    private SparkPIDController pid;

    public double freakLevel = 0;

    public BaseMotorNEO(int CANID) {
        //System.out.println("**** Activating SparkMAX NEO CANID:" + CANID);

        motorNEO = new CANSparkMax(CANID, MotorType.kBrushless);

        driveEncoder = motorNEO.getEncoder();
        angleEncoderRel = motorNEO.getEncoder();
        angleEncoder = motorNEO.getAbsoluteEncoder(Type.kDutyCycle);

        //this.CANID=CANID;



    }

    public static double calculateMetersPerRotation(
      double wheelDiameter, double driveGearRatio, double pulsePerRotation)
    {
    return (Math.PI * wheelDiameter) / (driveGearRatio * pulsePerRotation);
    }

    public static double calculateDegreesPerSteeringRotation(
      double angleGearRatio, double pulsePerRotation)
    {
        return 360 / (angleGearRatio * pulsePerRotation);
    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {



        motorNEO.restoreFactoryDefaults();
        motorNEO.clearFaults();

        motorNEO.setInverted(c.isDriveMotorInverted());
        //driveEncoder.setInverted(c.getDriveMotorSensorPhase());

        pid = motorNEO.getPIDController();

        driveEncoder.setPositionConversionFactor((calculateMetersPerRotation(
            SwerveChassis.WHEEL_DIAMETER,
            SwerveChassis.DRIVE_GEAR_RATIO,
            NEOSwerveConfiguration.DRIVE_PULSE_PER_ROTATION)));
        driveEncoder.setVelocityConversionFactor((calculateMetersPerRotation(
            SwerveChassis.WHEEL_DIAMETER,
            SwerveChassis.DRIVE_GEAR_RATIO,
            NEOSwerveConfiguration.DRIVE_PULSE_PER_ROTATION)) / 60);

        motorNEO.setCANTimeout(30);

        motorNEO.enableVoltageCompensation(NEOSwerveConfiguration.nominalVoltage);
        motorNEO.setSmartCurrentLimit(NEOSwerveConfiguration.driveMotorCurrentLimit);
        motorNEO.setOpenLoopRampRate(NEOSwerveConfiguration.rampRate);
        motorNEO.setClosedLoopRampRate(NEOSwerveConfiguration.rampRate);

        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(NEOSwerveConfiguration.minInput);
        pid.setPositionPIDWrappingMaxInput(NEOSwerveConfiguration.maxInput);


        pid.setP(NEOAngle.kP);
        pid.setI(NEOAngle.kI);
        pid.setD(NEOAngle.kD);
        pid.setFF(NEOAngle.kF);
        pid.setIZone(NEOAngle.kiz);
        pid.setOutputRange(NEOAngle.outputMin, NEOAngle.outputMax);


        motorBrakeMode();




        motorNEO.burnFlash();

    }

    public void configureAngleMotor(SwerveModuleConstants c) {

        this.cAngle=c;

        motorNEO.restoreFactoryDefaults();
        motorNEO.clearFaults();
        
        motorNEO.setInverted(c.isAngleMotorInverted());


        pid = motorNEO.getPIDController();

        pid.setFeedbackDevice(angleEncoder);

        angleEncoder.setPositionConversionFactor(NEOSwerveConfiguration.maxInput);
        angleEncoder.setVelocityConversionFactor(NEOSwerveConfiguration.maxInput);

        angleEncoder.setInverted(true);

        
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(NEOSwerveConfiguration.minInput);
        pid.setPositionPIDWrappingMaxInput(NEOSwerveConfiguration.maxInput);
        
        // Configure PID values

        pid.setP(NEOAngle.kP);
        pid.setI(NEOAngle.kI);
        pid.setD(NEOAngle.kD);
        pid.setFF(NEOAngle.kF);
        pid.setOutputRange(NEOAngle.outputMin, NEOAngle.outputMax);

        motorNEO.setIdleMode(IdleMode.kBrake);


        motorNEO.burnFlash();

        
        motorBrakeMode();

        setEncoderforWheelCalibration(c);

    }

    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    public double getAngleEncoderPosition() {
        return angleEncoder.getPosition();
    }

    public double getAngleEncoderPositionCorrected() {
        return angleEncoder.getPosition() - getOffsetInRadians();
    }

    public double getDriveEncoderVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getFreakiness() {
        return driveEncoder.getVelocity();
      }

    public double getAngleEncoderVelocity() {
        return angleEncoder.getVelocity();
    }

    public double getDriveEncoderPositionSI() {
       
        return getDriveEncoderPosition()*Constants.SwerveChassis.NEOSwerveConfiguration.metersPerTick;
    }
    
    public double getOffsetInRadians () {
        return cAngle.getAngleOffset()*(NEOSwerveConfiguration.maxInput/360); // = 2pi/360 
    }

    public double getAngleEncoderPositionSI() {
      
        return getAngleEncoderPosition()*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public double getAngleEncoderPositionSICorrected() {
      
        return ((getAngleEncoderPosition() - (getOffsetInRadians()))*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick)%360;
    }

    public double getDriveEncoderVelocitySI() {
   
        return getDriveEncoderVelocity()*Constants.SwerveChassis.NEOSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
       
        return getAngleEncoderVelocity()*Constants.SwerveChassis.NEOSwerveConfiguration.degreePerTick;
    }

    public void setAngleMotorChassisAngleSI(double angle) {
        //System.out.println("T:"+degreesToTicks(angle) + " A: "+angle);
        motorNEO.getPIDController().setReference(degreesToTicks(angle), ControlType.kPosition);
    }

    public void testMotorApplyPower(double power) {
        motorNEO.set(power);
    }

    public void applyPower(double power) {
        motorNEO.set(power);
    }

    private void motorBrakeMode(){
        motorNEO.setIdleMode(IdleMode.kBrake);
    }

    

    private double degreesToTicks(double degrees) {
        return ((degrees+cAngle.getAngleOffset()) / NEOSwerveConfiguration.degreePerTick)  
         % 
        (NEOSwerveConfiguration.ticksPerFullRotation);
    }

     
    public void setEncoderforWheelCalibration(SwerveModuleConstants c) {
        double difference = (getAngleEncoderPosition() - c.getAngleOffset())
                / NEOSwerveConfiguration.degreePerTick; // cancoder returns Abs value in Degrees
        double encoderSetting = 0.0;

        //System.out.println(c.getAngleMotorID()+"#"+getCancoderAbsEncoderValue()+"#"+c.getAngleOffset()+"#");

        if (difference < 0) {
            difference += NEOSwerveConfiguration.ticksPerFullRotation;
        }

        if (difference <= NEOSwerveConfiguration.ticksPerFullRotation / 2) {
            encoderSetting = difference;

        } else {
            encoderSetting = difference - NEOSwerveConfiguration.ticksPerFullRotation;
        }

        angleEncoderRel.setPosition(encoderSetting);

        System.out.println("Set encoder for motor " + c.getAngleMotorID() + " to " + encoderSetting);

    }


}
