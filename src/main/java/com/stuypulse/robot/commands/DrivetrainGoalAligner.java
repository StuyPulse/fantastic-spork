package com.stuypulse.robot.commands;

import com.stuypulse.robot.Constants.Alignment;
import com.stuypulse.stuylib.network.limelight.Limelight;

/**
 * The drivetrain goal aligner is an aligner that uses the limelight to send
 * instructions to the drivetrain on how it should move to align with the
 * target.
 */
public class DrivetrainGoalAligner implements DrivetrainAlignmentCommand.Aligner {

    private double mTargetDistance;

    public DrivetrainGoalAligner(double distance) {
        mTargetDistance = distance;
        
        // Turn on LEDs for CV
        Limelight.setLEDMode(Limelight.LEDMode.FORCE_ON);
    }

    public double getSpeedError() {
        // TODO: have CV replace this command
        double goal_pitch = Limelight.getTargetYAngle() + Alignment.Measurements.Limelight.kPitch;
        double goal_height = Alignment.Measurements.kGoalHeight - Alignment.Measurements.Limelight.kHeight;
        double goal_dist = goal_height / Math.tan(Math.toRadians(goal_pitch))
                - Alignment.Measurements.Limelight.kDistance;

        // Return the error from the target distance
        return goal_dist - mTargetDistance;
    }

    public double getAngleError() {
        // TODO: have CV replace this command
        return Limelight.getTargetXAngle() + Alignment.Measurements.Limelight.kYaw;
    }
}