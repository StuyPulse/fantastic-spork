/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.commands;

import com.stuypulse.robot.Constants;
import com.stuypulse.robot.subsystems.Drivetrain;

/** Command that aligns with the target */
public class DrivetrainGoalCommand extends DrivetrainAlignmentCommand {

    /**
     * Create a drivetrain goal command.
     * 
     * @param drivetrain drivetrain to do movements on
     * @param goalAligner a goal aligner class -- should be using offsets from the limelight
     */
    public DrivetrainGoalCommand(Drivetrain drivetrain, DrivetrainGoalAligner goalAligner) {
        super(drivetrain, goalAligner);
        setMaxSpeed(Constants.Alignment.Speed.LIMELIGHT_MAX_SPEED);
        useMinTime();
    }

    /**
     * Create a drivetrain goal command.
     * 
     * @param drivetrain drivetrain to do movements on
     * @param distance distance to stop in front of the target
     */
    public DrivetrainGoalCommand(Drivetrain drivetrain, Number distance) {
        this(drivetrain, new DrivetrainGoalAligner(distance));
    }
}
