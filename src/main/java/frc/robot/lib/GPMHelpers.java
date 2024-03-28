package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class GPMHelpers {

    public InterpolatingDoubleTreeMap GPM_0_Angle = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_0_IntakePower = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_0_ShooterPower = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap Measured_Shooter_Speeds_From_Power = new InterpolatingDoubleTreeMap();

    public GPMHelpers() {
        setGPM0Angle();
        setGPM0IntakePower();
        setGPM0ShooterPower();
        setMeasuredShootingSpeedsFromPower();
    }

    public double getAngleTouchingAmp() {
        return 0.0;
    }

    public double getAngleBeforeTouchingAmp() {
        return 0.0;
    }

    public double getIntakePowerTouchingAmp() {
        return 0.0;
    }

    public double getShooterPowerTouchingAmp() {
        return 0.0;
    }

    public void setGPM0Angle() {
        GPM_0_Angle.put(0.0, 0.0);
        GPM_0_Angle.put(0.25, 0.0);
        GPM_0_Angle.put(0.5, 0.0);
        GPM_0_Angle.put(0.75, 0.0);
        GPM_0_Angle.put(1.0, 0.0);
        GPM_0_Angle.put(1.25, 0.0);
        GPM_0_Angle.put(1.5, 0.0);
        GPM_0_Angle.put(1.75, 0.0);
        GPM_0_Angle.put(2.0, 0.0);
        GPM_0_Angle.put(2.25, 0.0);
        GPM_0_Angle.put(2.5, 0.0);
        GPM_0_Angle.put(2.75, 0.0);
        GPM_0_Angle.put(3.0, 0.0);
        GPM_0_Angle.put(3.25, 0.0);
        GPM_0_Angle.put(3.5, 0.0);
        GPM_0_Angle.put(3.75, 0.0);
        GPM_0_Angle.put(4.0, 0.0);
    }

    public void setGPM0IntakePower() {
        GPM_0_IntakePower.put(0.0, 0.0);
        GPM_0_IntakePower.put(0.25, 0.0);
        GPM_0_IntakePower.put(0.5, 0.0);
        GPM_0_IntakePower.put(0.75, 0.0);
        GPM_0_IntakePower.put(1.0, 0.0);
        GPM_0_IntakePower.put(1.25, 0.0);
        GPM_0_IntakePower.put(1.5, 0.0);
        GPM_0_IntakePower.put(1.75, 0.0);
        GPM_0_IntakePower.put(2.0, 0.0);
        GPM_0_IntakePower.put(2.25, 0.0);
        GPM_0_IntakePower.put(2.5, 0.0);
        GPM_0_IntakePower.put(2.75, 0.0);
        GPM_0_IntakePower.put(3.0, 0.0);
        GPM_0_IntakePower.put(3.25, 0.0);
        GPM_0_IntakePower.put(3.5, 0.0);
        GPM_0_IntakePower.put(3.75, 0.0);
        GPM_0_IntakePower.put(4.0, 0.0);
    }

    public void setGPM0ShooterPower() {

        GPM_0_ShooterPower.put(0.0, 0.0);
        GPM_0_ShooterPower.put(0.25, 0.0);
        GPM_0_ShooterPower.put(0.5, 0.0);
        GPM_0_ShooterPower.put(0.75, 0.0);
        GPM_0_ShooterPower.put(1.0, 0.0);
        GPM_0_ShooterPower.put(1.25, 0.0);
        GPM_0_ShooterPower.put(1.5, 0.0);
        GPM_0_ShooterPower.put(1.75, 0.0);
        GPM_0_ShooterPower.put(2.0, 0.0);
        GPM_0_ShooterPower.put(2.25, 0.0);
        GPM_0_ShooterPower.put(2.5, 0.0);
        GPM_0_ShooterPower.put(2.75, 0.0);
        GPM_0_ShooterPower.put(3.0, 0.0);
        GPM_0_ShooterPower.put(3.25, 0.0);
        GPM_0_ShooterPower.put(3.5, 0.0);
        GPM_0_ShooterPower.put(3.75, 0.0);
        GPM_0_ShooterPower.put(4.0, 0.0);
    }

    public void setMeasuredShootingSpeedsFromPower() {   
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(0.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(0.25), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(0.5), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(0.75), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(1.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(1.25), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(1.5), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(1.75), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(2.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(2.25), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(2.5), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(2.75), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(3.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(3.25), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(3.5), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(3.75), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(4.0), 0.0);
    }

    public double getMeasuredShootingSpeedFromPower(double power)   {
        return Measured_Shooter_Speeds_From_Power.get(power);
    }

    public double getGPM0Angle(double distance){
        return GPM_0_Angle.get(distance);
    }

    public double getGPM0IntakePower(double distance){
        return GPM_0_IntakePower.get(distance);
    }

    public double getGPM0ShooterPower(double distance){
        return GPM_0_ShooterPower.get(distance);
    }

}