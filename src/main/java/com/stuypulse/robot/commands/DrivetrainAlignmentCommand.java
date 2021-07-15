/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.IFilterGroup;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.util.StopWatch;


import com.stuypulse.robot.Constants.Alignment;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.util.IFuser;

/**
 * Drivetrain Alignment Command takes in a drivetrain, an aligner, and two controllers. This lets
 * you align the robot with whatever controllers you want. Most commonly, a
 * DrivetrainPIDAlignmentCommand is used instead as it automatically provides the controllers for
 * you.
 */
public class DrivetrainAlignmentCommand extends DrivetrainCommand {

    /**
     * This interface allows you to create classes that instruct the drivetrain to move based off of
     * error for speed and angle. If you use an aligner to define your class, you can do things like
     * auto tune.
     */
    public interface Aligner {
        // Called when command initialize is called,
        // Useful for relative encoder commands
        public default void init() {}

        // The amount of positional error
        public default double getSpeedError() {
            return 0.0;
        }

        // The amount of angular error
        public default Angle getAngleError() {
            return Angle.fromDegrees(0);
        }
    }

    // Max speed for the robot
    private double maxSpeed;

    // Controllers for Alignment
    protected Controller speed;
    protected Controller angle;

    // Distance that the command will try to align with
    protected Aligner aligner;

    // Variables for fusing the alignment
    // data with the encoder data
    private double initAngleMeasurement; 
    private double initSpeedMeasurement;

    private IFuser angleFuser;
    private IFuser speedFuser;

    // Used to check timeout of alignment
    private StopWatch timer;

    // Misc Settings
    private boolean continuous; // Removes check for velocity
    private boolean neverFinish; // Waits to be interrupted
    private boolean minTime; // Waits to be interrupted

    /**
     * This creates a command that aligns the robot
     *
     * @param drivetrain Drivetrain used by command to move
     * @param distance target distance for robot to drive to
     * @param speed controller used to align distance
     * @param angle controller used to align the angle
     */
    public DrivetrainAlignmentCommand(
            Drivetrain drivetrain, Aligner aligner, Controller speed, Controller angle) {
        // Pass Drivetrain to the super class
        super(drivetrain);

        // Max speed
        this.maxSpeed = 1.0;

        // Initialize PID Controller for Speed
        this.speed = speed;

        // Initialize PID Controller for Angle
        this.angle = angle;

        // Target distance for the Alignment Command
        this.aligner = aligner;

        // Timer used to check when to update the errors
        this.initAngleMeasurement = 0;
        this.initSpeedMeasurement = 0;

        this.speedFuser = new IFuser(
            Alignment.SENSOR_FUSION_RC,
            aligner::getSpeedError,
            () -> initSpeedMeasurement - drivetrain.getDistance()
        );
            
        this.angleFuser = new IFuser(
            Alignment.SENSOR_FUSION_RC,
            () -> aligner.getAngleError().toDegrees(),
            () -> initAngleMeasurement - drivetrain.getRawAngle()
        );

        // Used to check the alignment time.
        this.timer = new StopWatch();

        // Normally end the command once aligned
        this.neverFinish = false;
        this.continuous = false;
        this.minTime = false;
    }

    /**
     * This creates a command that aligns the robot
     *
     * @param drivetrain Drivetrain used by command to move
     * @param distance target distance for robot to drive to
     */
    public DrivetrainAlignmentCommand(Drivetrain drivetrain, Aligner aligner) {
        this(drivetrain, aligner, Alignment.Speed.getPID(), Alignment.Angle.getPID());
    }

    // Set the speed of the movement command
    public DrivetrainAlignmentCommand setMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    // Make the command never finish
    public DrivetrainAlignmentCommand setNeverFinish() {
        this.neverFinish = true;
        return this;
    }

    // Make command not check for velocity when finishing
    public DrivetrainAlignmentCommand setContinuous() {
        this.continuous = true;
        return this;
    }

    // Make command not check for velocity when finishing
    public DrivetrainAlignmentCommand useMinTime() {
        this.minTime = true;
        return this;
    }

    // Set the gear and other things when initializing
    public void initialize() {
        aligner.init();
        timer.reset();

        this.speed.setErrorFilter(new LowPassFilter(Alignment.Speed.IN_SMOOTH_FILTER));
        this.speed.setOutputFilter(
                new IFilterGroup(
                        (x) -> SLMath.clamp(x, maxSpeed),
                        new LowPassFilter(Alignment.Speed.OUT_SMOOTH_FILTER)));

        this.angle.setErrorFilter(new LowPassFilter(Alignment.Angle.IN_SMOOTH_FILTER));
        this.angle.setOutputFilter(
                new IFilterGroup(new LowPassFilter(Alignment.Angle.OUT_SMOOTH_FILTER)));

        this.initAngleMeasurement = drivetrain.getRawAngle();
        this.initSpeedMeasurement = drivetrain.getDistance();
    }

    // Get distance left to travel
    public double getSpeedError() {
        // The speed fuser combines the low frequency of the raw aligner
        // and the high frequencies of the encoder data using high / low pass filters
        return speedFuser.get();
    }

    // Get angle left to turn
    public Angle getAngleError() {
        // The speed fuser combines the low frequency of the raw aligner
        // and the high frequencies of the gyroscope data using high / low pass filters
        return Angle.fromDegrees(angleFuser.get());
    }

    // Speed robot should move
    public double getSpeed() {
        // The more unaligned the robot is, the less it moves
        double s = 1.5 - Math.abs(angle.getError()) / Alignment.Angle.MAX_ANGLE_ERROR;
        return speed.update(this.getSpeedError()) * SLMath.clamp(s, 0, 1.0);
    }

    // Angle robot has to turn
    public double getAngle() {
        return angle.update(getAngleError().toDegrees());
    }

    // Alignment must use low gear
    public Drivetrain.Gear getGear() {
        return Drivetrain.Gear.LOW;
    }

    // Aligning doesn't need to use curvature drive
    // Arcade drive is better for non humans
    public boolean useCurvatureDrive() {
        return false;
    }

    // Execute loop while also updating PID controllers
    public void execute() {
        super.execute();
    }

    // Command is finished if all of the errors are small enough
    public boolean isFinished() {
        // If you do not want the command to automatically finish
        if (neverFinish) {
            return false;
        }

        // Check if the aligner hasn't run for long enough
        if (this.minTime && timer.getTime() < Alignment.MIN_ALIGNMENT_TIME) {
            return false;
        }

        // If continuous, do not check for velocity
        if (this.continuous) {
            return (speed.isDone(Alignment.Speed.MAX_SPEED_ERROR * 2.5)
                    && angle.isDone(Alignment.Angle.MAX_ANGLE_ERROR * 1.5));
        } else {
            return (speed.isDone(Alignment.Speed.MAX_SPEED_ERROR, Alignment.Speed.MAX_SPEED_VEL)
                    && angle.isDone(
                            Alignment.Angle.MAX_ANGLE_ERROR, Alignment.Angle.MAX_ANGLE_VEL));
        }
    }

    // Turn limelight off when no longer aligning due to rules
    public void end(boolean interrupted) {
        // Limelight.getInstance().setLEDMode(Limelight.LEDMode.FORCE_OFF);
    }
}
