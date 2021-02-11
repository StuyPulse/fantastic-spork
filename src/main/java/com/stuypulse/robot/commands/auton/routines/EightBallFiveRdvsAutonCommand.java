package com.stuypulse.robot.commands.auton.routines;

import com.stuypulse.robot.Constants.AutoSettings;
import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EightBallFiveRdvsAutonCommand extends SequentialCommandGroup {
    public EightBallFiveRdvsAutonCommand(Drivetrain drivetrain, Intake intake) {
        addCommands(
                new DrivetrainGoalCommand(drivetrain, AutoSettings.SHOOT_FROM_START_TO_GOAL),
                // TODO: Add shoot 3

                // Move forward
                new DrivetrainMovementCommand(drivetrain, 0, AutoSettings.DISTANCE_FROM_START_TO_RDVS),
                new DrivetrainMovementCommand(drivetrain, AutoSettings.ANGLE_FROM_START_POINT_TO_THREE_BALL),
                new IntakeAcquireCommand(intake),
                new DrivetrainMovementCommand(drivetrain, 0, AutoSettings.DISTANCE_FOR_THREE_BALLS_IN_RDVS),
                new DrivetrainMovementCommand(drivetrain, AutoSettings.ANGLE_FROM_THREE_BALL_TO_TWO_BALL,
                        AutoSettings.DISTANCE_FROM_THREE_BALL_TO_TWO_BALL),
                new DrivetrainMovementCommand(drivetrain, 180), new DrivetrainStopCommand(drivetrain),
                new DrivetrainAlignmentCommand(drivetrain, new DrivetrainGoalAligner(20))
        // Add Shoot code
        );
    }
}