/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.stuypulse.robot.Constants.DrivetrainSettings;
import com.stuypulse.robot.Constants.Ports;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.stuypulse.robot.util.Encoder;

public class Drivetrain extends SubsystemBase {

    public enum Gear {

        // Create all the gear settings with their ratios
    
        HIGH(DrivetrainSettings.Encoders.HIGH_GEAR_DISTANCE_PER_ROTATION),
        LOW(DrivetrainSettings.Encoders.LOW_GEAR_DISTANCE_PER_ROTATION);
    
        // Store the gear ratio
    
        private final Number ratio;
    
        private Gear(Number ratio) {
            this.ratio = ratio;
        }
    
        // Functions to use the gear ratio
    
        public Number getRatio() {
            return this.ratio;
        }
    
        public double getScaledDistance(double rotations) {
            return getRatio().doubleValue() * rotations;
        }
    
    }
    
    // An array of motors on the left and right side of the drive train
    private CANSparkMax[] leftMotors;
    private CANSparkMax[] rightMotors;

    // An encoder for each side of the drive train
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    // DifferentialDrive and Gear Information
    private Gear gear;
    private Solenoid gearShift;
    private DifferentialDrive drivetrain;

    // NAVX for Gyro
    private AHRS navx;

    // Odometry
    private DifferentialDriveOdometry odometry;
    private Field2d field;

    // State Variable
    private boolean isAligned;

    public Drivetrain() {
        // Add Motors to list
        leftMotors =
                new CANSparkMax[] {
                    new CANSparkMax(Ports.Drivetrain.LEFT_TOP, MotorType.kBrushless),
                    new CANSparkMax(Ports.Drivetrain.LEFT_BOTTOM, MotorType.kBrushless)
                };

        rightMotors =
                new CANSparkMax[] {
                    new CANSparkMax(Ports.Drivetrain.RIGHT_TOP, MotorType.kBrushless),
                    new CANSparkMax(Ports.Drivetrain.RIGHT_BOTTOM, MotorType.kBrushless)
                };

        // Gear shifiting
        gearShift = new Solenoid(Ports.Drivetrain.GEAR_SHIFT);
        setGear(Gear.HIGH);

        // Create list of encoders based on motors
        leftEncoder = new Encoder(leftMotors[0].getEncoder());
        rightEncoder = new Encoder(rightMotors[0].getEncoder());
            
        resetEncoders();

        // Make differential drive object
        drivetrain =
                new DifferentialDrive(
                        new SpeedControllerGroup(leftMotors),
                        new SpeedControllerGroup(rightMotors));


        // Initialize NAVX
        navx = new AHRS(SPI.Port.kMXP);

        // Initialize Odometry
        odometry =
                new DifferentialDriveOdometry(
                        DrivetrainSettings.Odometry.STARTING_ANGLE,
                        DrivetrainSettings.Odometry.STARTING_POSITION);
        field = new Field2d();

        // Configure Motors and Other Things
        setInverted(DrivetrainSettings.IS_INVERTED);
        setSmartCurrentLimit(DrivetrainSettings.CURRENT_LIMIT);
        leftMotors[0].setIdleMode(IdleMode.kBrake);
        leftMotors[1].setIdleMode(IdleMode.kBrake);
        rightMotors[0].setIdleMode(IdleMode.kBrake);
        rightMotors[1].setIdleMode(IdleMode.kBrake);

        // Add Children to Subsystem
        addChild("Gear Shift", gearShift);
        addChild("Differential Drive", drivetrain);
        addChild("NavX", navx);
        addChild("Field Map", field);
    }

    /***********************
     * MOTOR CONFIGURATION *
     ***********************/

    // Set the smart current limit of all the motors
    public void setSmartCurrentLimit(int limit) {
        for (CANSparkMax motor : leftMotors) {
            motor.setSmartCurrentLimit(limit);
        }

        for (CANSparkMax motor : rightMotors) {
            motor.setSmartCurrentLimit(limit);
        }
    }

    // Set the idle mode of the all the motors
    public void setIdleMode(IdleMode mode) {
        for (CANSparkMax motor : leftMotors) {
            motor.setIdleMode(mode);
        }

        for (CANSparkMax motor : rightMotors) {
            motor.setIdleMode(mode);
        }
    }

    // Set isInverted of all the motors
    public void setInverted(boolean inverted) {
        for (CANSparkMax motor : leftMotors) {
            motor.setInverted(inverted);
        }

        for (CANSparkMax motor : rightMotors) {
            motor.setInverted(inverted);
        }
    }

    /*****************
     * Gear Shifting *
     *****************/

    // Gets the current gear the robot is in
    public Gear getGear() {
        return gear;
    }

    // Sets the current gear the robot is in
    public void setGear(Gear gear) {
        this.gear = gear;
        gearShift.set(gear == Gear.HIGH);
    }

    // Sets robot into low gear
    public void setLowGear() {
        setGear(Gear.LOW);
    }

    // Sets robot into high gear
    public void setHighGear() {
        setGear(Gear.HIGH);
    }

    /********
     * NAVX *
     ********/

    // Gets current Angle of the Robot as a double (contiuous / not +-180)
    public double getRawAngle() {
        return navx.getAngle();
    }

    // Gets current Angle of the Robot
    public Angle getAngle() {
        return Angle.fromDegrees(getRawAngle());
    }

    private void resetNavX() {
        navx.reset();
    }

    /*********************
     * ENCODER FUNCTIONS *
     *********************/

    // Distance

    public double getLeftDistance() {
        return leftEncoder.getDistance() * DrivetrainSettings.Encoders.LEFT_YEILD;
    }

    public double getRightDistance() {
        return rightEncoder.getDistance() * DrivetrainSettings.Encoders.RIGHT_YEILD;
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    private void updateDistance() {
        leftEncoder.periodic(getGear());
        rightEncoder.periodic(getGear());
    }

    private void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    // Velocity
    public double getLeftVelocity() {
        return leftEncoder.getRPM() * DrivetrainSettings.Encoders.LEFT_YEILD;
    }

    public double getRightVelocity() {
        return rightEncoder.getRPM() * DrivetrainSettings.Encoders.RIGHT_YEILD;
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /**********************
     * ODOMETRY FUNCTIONS *
     **********************/

    private void updateOdometry() {
        odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public Rotation2d getRotation2d() {
        // TODO: check if this needs to be negative
        return getAngle().negative().getRotation2d();
    }

    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    private void resetOdometer(Pose2d start) {
        odometry.resetPosition(start, DrivetrainSettings.Odometry.STARTING_ANGLE);
    }

    /************************
     * OVERALL SENSOR RESET *
     ************************/

    public void reset(Pose2d location) {
        resetNavX();
        resetEncoders();
        resetOdometer(location);
    }

    public void reset() {
        reset(DrivetrainSettings.Odometry.STARTING_POSITION);
    }

    /*********************
     * VOLTAGE FUNCTIONS *
     *********************/

    public double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public double getLeftVoltage() {
        return leftMotors[0].get() * getBatteryVoltage() / DrivetrainSettings.LEFT_VOLTAGE_MUL;
    }

    public double getRightVoltage() {
        return rightMotors[0].get() * getBatteryVoltage() / DrivetrainSettings.RIGHT_VOLTAGE_MUL;
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        for (SpeedController motor : leftMotors) {
            motor.setVoltage(leftVolts * DrivetrainSettings.LEFT_VOLTAGE_MUL);
        }

        for (SpeedController motor : rightMotors) {
            motor.setVoltage(rightVolts * DrivetrainSettings.RIGHT_VOLTAGE_MUL);
        }

        drivetrain.feed();
    }

    /********************
     * DRIVING COMMANDS *
     ********************/

    // Stops drivetrain from moving
    public void stop() {
        drivetrain.stopMotor();
    }

    // Drives using tank drive
    public void tankDrive(double left, double right) {
        drivetrain.tankDrive(left, right, false);
    }

    // Drives using arcade drive
    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation, false);
    }

    // Drives using curvature drive algorithm
    public void curvatureDrive(double speed, double rotation, boolean quickturn) {
        drivetrain.curvatureDrive(speed, rotation, quickturn);
    }

    // Drives using curvature drive algorithm with automatic quick turn
    public void curvatureDrive(double speed, double rotation) {
        if (Math.abs(speed) < DrivetrainSettings.QUICKTURN_THRESHOLD.get()) {
            curvatureDrive(speed, rotation * DrivetrainSettings.QUICKTURN_SPEED.get(), true);
        } else {
            curvatureDrive(speed, rotation, false);
        }
    }

    /*******************
     * STATE FUNCTIONS *
     *******************/

    public void setIsAligned(boolean aligned) {
        isAligned = aligned;
    }

    public boolean getIsAligned() {
        return isAligned;
    }

    /*********************
     * DEBUG INFORMATION *
     *********************/

    @Override
    public void periodic() {
        // Encoder data (requires periodic updating)
        updateDistance();

        // Odemetry (and related debug information)
        updateOdometry();
        field.setRobotPose(getPose());

        // Smart Dashboard Information
        SmartDashboard.putData("Drivetrain/Field", field);
        SmartDashboard.putString(
                "Drivetrain/Current Gear", getGear().equals(Gear.HIGH) ? "High Gear" : "Low Gear");
        SmartDashboard.putNumber("Drivetrain/Odometer X Position (m)", getPose().getX());
        SmartDashboard.putNumber("Drivetrain/Odometer Y Position (m)", getPose().getY());
        SmartDashboard.putNumber(
                "Drivetrain/Odometer Rotation (deg)", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Drivetrain/Motor Voltage Left (V)", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/Motor Voltage Right (V)", getRightVoltage());

        SmartDashboard.putNumber("Drivetrain/Distance Traveled (m)", getDistance());
        SmartDashboard.putNumber("Drivetrain/Distance Traveled Left (m)", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain/Distance Traveled Right (m)", getRightDistance());

        SmartDashboard.putNumber("Drivetrain/Velocity (m per s)", getVelocity());
        SmartDashboard.putNumber("Drivetrain/Velocity Left (m per s)", getLeftVelocity());
        SmartDashboard.putNumber("Drivetrain/Velocity Right (m per s)", getRightVelocity());

        SmartDashboard.putNumber("Drivetrain/Angle NavX (deg)", getAngle().toDegrees());
        SmartDashboard.putBoolean("Drivetrain/Is Aligned", getIsAligned());
    }
}
