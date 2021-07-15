package com.stuypulse.robot.util;

import com.revrobotics.CANEncoder;
import com.stuypulse.robot.subsystems.Drivetrain.Gear;

/**
 * TODO: add javadocs !!!
 * TODO: Velocity needs major checking !!!
 * TODO: make this an encoder subsystem  ???
 * 
 * @author Myles Pasetsky (@selym3)
 */
public class Encoder {
    
    private CANEncoder encoder;

    private double accumulated;
    private double lastRotations;

    public Encoder(CANEncoder encoder) {
        this.encoder = encoder; 
        reset();
    }

    public double getRotations() {
        return encoder.getPosition();
    }

    public double getDistance() {
        return accumulated;
    }
    
    public double getRPM() {
        return encoder.getVelocity();
    }

    // public double getVelocity() {
    //     return (implement delta accumulated)
    // }

    public void reset() {
        // Reset encoder tracking
        encoder.setPositionConversionFactor(1.0); // <-- always ensure this is true
        encoder.setPosition(0);

        // Reset distance tracking
        accumulated = 0.0;
        lastRotations = 0.0;
    }

    public void periodic(Gear gear) {
        final double nowRotations = getRotations();

        double rotations = nowRotations - lastRotations;
        double distance = gear.getScaledDistance(rotations);

        accumulated += distance;
        lastRotations = nowRotations;
    }

}
