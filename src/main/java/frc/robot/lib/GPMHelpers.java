package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class GPMHelpers {

    public InterpolatingDoubleTreeMap GPM_0_Angle = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_0_ShooterPower = new InterpolatingDoubleTreeMap();

    public GPMHelpers() {
        setGPM0Angle();
        setGPM0ShooterPower();
    }

    public void setGPM0Angle() {
        GPM_0_Angle.put(0.0, -39.5);
        // GPM_0_Angle.put(0.25, 0.0);
        // GPM_0_Angle.put(0.5, 0.0);
        // GPM_0_Angle.put(0.75, 0.0);
        GPM_0_Angle.put(1.0, -34.0);
        // GPM_0_Angle.put(1.25, 0.0);
        // GPM_0_Angle.put(1.5, 0.0);
        // GPM_0_Angle.put(1.75, 0.0);
        GPM_0_Angle.put(2.0, -32.0);
        // GPM_0_Angle.put(2.25, 0.0);
        // GPM_0_Angle.put(2.5, 0.0);
        // GPM_0_Angle.put(2.75, 0.0);
        GPM_0_Angle.put(3.0, -31.0);
        // GPM_0_Angle.put(3.25, 0.0);
        // GPM_0_Angle.put(3.5, 0.0);
        // GPM_0_Angle.put(3.75, 0.0);
        GPM_0_Angle.put(4.0, -30.0);
    }

    public void setGPM0ShooterPower() {

        GPM_0_ShooterPower.put(0.0, 1500.0);
        // GPM_0_ShooterPower.put(0.25, 0.0);
        // GPM_0_ShooterPower.put(0.5, 0.0);
        // GPM_0_ShooterPower.put(0.75, 0.0);
        GPM_0_ShooterPower.put(1.0, 1500.0);
        // GPM_0_ShooterPower.put(1.25, 0.0);
        // GPM_0_ShooterPower.put(1.5, 0.0);
        // GPM_0_ShooterPower.put(1.75, 0.0);
        GPM_0_ShooterPower.put(2.0, 1700.0);
        // GPM_0_ShooterPower.put(2.25, 0.0);
        // GPM_0_ShooterPower.put(2.5, 0.0);
        // GPM_0_ShooterPower.put(2.75, 0.0);
        GPM_0_ShooterPower.put(3.0, 1900.0);
        // GPM_0_ShooterPower.put(3.25, 0.0);
        // GPM_0_ShooterPower.put(3.5, 0.0);
        // GPM_0_ShooterPower.put(3.75, 0.0);
        GPM_0_ShooterPower.put(4.0, 2000.0);
    }

    public double getGPM0Angle(double distance){
        return GPM_0_Angle.get(distance);
    }

    public double getGPM0ShooterPower(double distance){
        return GPM_0_ShooterPower.get(distance);
    }

}