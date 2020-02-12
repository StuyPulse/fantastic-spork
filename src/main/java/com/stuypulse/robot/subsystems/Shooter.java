package com.stuypulse.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.Constants;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Shooter Motors
    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;
    private CANSparkMax middleShooterMotor;

    // Shooter Encoders
    private CANEncoder leftShooterEncoder;
    private CANEncoder rightShooterEncoder; 
    private CANEncoder middleShooterEncoder;

    // Feeder Motor
    private CANSparkMax feederMotor;

    // Hood Solenoid
    private Solenoid hoodSolenoid;

    // SpeedControllerGroup
    private SpeedControllerGroup shooterMotors; 

    // SmartNumbers for SmartDashboard
    private SmartNumber targetVelocity;
    private SmartNumber currentVelocity;

    public Shooter() {
        leftShooterMotor = new CANSparkMax(Constants.LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(Constants.RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
        middleShooterMotor = new CANSparkMax(Constants.MIDDLE_SHOOTER_MOTOR_PORT, MotorType.kBrushless);

        leftShooterMotor.setInverted(true); 

        leftShooterEncoder = new CANEncoder(leftShooterMotor);
        rightShooterEncoder = new CANEncoder(rightShooterMotor);
        middleShooterEncoder = new CANEncoder(middleShooterMotor);

        feederMotor = new CANSparkMax(Constants.FEEDER_MOTOR_PORT, MotorType.kBrushless);

        hoodSolenoid = new Solenoid(Constants.HOOD_SOLENOID_PORT);

        shooterMotors = new SpeedControllerGroup(leftShooterMotor, rightShooterMotor, middleShooterMotor);

        targetVelocity = new SmartNumber("Shooter Target Vel", 69420);
        currentVelocity = new SmartNumber("Shooter Current Vel", -1);
    }
    
    public double getRawMedianShooterVelocity() {
        double[] speeds = {
            leftShooterEncoder.getVelocity(),
            middleShooterEncoder.getVelocity(),
            rightShooterEncoder.getVelocity()
        };
        
        Arrays.sort(speeds);

        return speeds[1];
    }

    public double getRawMaxShooterVelocity() {
        return Math.max(leftShooterEncoder.getVelocity(), Math.max(middleShooterEncoder.getVelocity(), rightShooterEncoder.getVelocity()));
    }

    public double getRawMinShooterVelocity() {
        return Math.min(leftShooterEncoder.getVelocity(), Math.min(middleShooterEncoder.getVelocity(), rightShooterEncoder.getVelocity()));
    }

    public double getCurrentShooterVelocityInRPM() {
        double speed = getRawMedianShooterVelocity() * Constants.SHOOTER_VELOCITY_EMPIRICAL_MULTIPLER;
        currentVelocity.set(speed);
        return speed;
    }

    public void setShooterSpeed(double speed) {
        shooterMotors.set(speed);
    }

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity.set(targetVelocity);
    }

    public double getTargetVelocity() {
        return this.targetVelocity.get();
    }

    public void reverseShooter() {
        shooterMotors.set(-1.0);
    }

    public void stopShooter() {
        shooterMotors.stopMotor();
    }

    public void runFeeder() {
        feederMotor.set(Constants.FEEDER_SPEED);
    }

    public void reverseFeed() {
        feederMotor.set(-1.0);
    }

    public void stopFeeder() {
        feederMotor.stopMotor();
    }
    
    public void extendHoodSolenoid() {
        hoodSolenoid.set(true);
    }

    public void retractHoodSolenoid() {
        hoodSolenoid.set(false);
    }

    public void setDefaultSolenoidPosition() {
        retractHoodSolenoid();
    }
}