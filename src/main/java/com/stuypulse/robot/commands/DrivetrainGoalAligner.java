/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import com.stuypulse.robot.Constants.Alignment;

/**
 * The drivetrain goal aligner is an aligner that uses the limelight to send instructions to the
 * drivetrain on how it should move to align with the target.
 */
public class DrivetrainGoalAligner implements DrivetrainAlignmentCommand.Aligner {

    protected Number distance;

    public DrivetrainGoalAligner(Number distance) {
        this.distance = distance;
    }

    public void init() {
        // Turn on LEDs for CV
        Limelight.getInstance().setLEDMode(Limelight.LEDMode.FORCE_ON);
    }

    public double getSpeedError() {
        if (distance.doubleValue() <= 0) {
            return 0;
        }

        if (Limelight.getInstance().getValidTarget()) {
            double goal_pitch =
                    Limelight.getInstance().getTargetYAngle()
                            + Alignment.Measurements.Limelight.PITCH;
            double goal_height =
                    Alignment.Measurements.GOAL_HEIGHT - Alignment.Measurements.Limelight.HEIGHT;
            double goal_dist =
                    goal_height / Math.tan(Math.toRadians(goal_pitch))
                            - Alignment.Measurements.Limelight.DISTANCE;

            if (goal_dist < Alignment.MIN_DISTANCE) {
                return 0;
            } else if (goal_dist > Alignment.MAX_DISTANCE) {
                return 0;
            }

            // Return the error from the target distance
            return 0 - (goal_dist - distance.doubleValue());
        } else {
            return 0;
        }
    }

    public Angle getAngleError() {
        if (Limelight.getInstance().getValidTarget()) {
            return Angle.fromDegrees(
                    Limelight.getInstance().getTargetXAngle()
                            + Alignment.Measurements.Limelight.YAW);
        } else {
            return Angle.fromDegrees(0);
        }
    }
}
