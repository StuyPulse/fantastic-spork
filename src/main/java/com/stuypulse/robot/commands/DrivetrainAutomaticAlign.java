package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.Shooter;

/**
 * Command that aligns with the goal based on the shooting range stored in the
 * shooter
 */
public class DrivetrainAutomaticAlign extends DrivetrainGoalCommand {

    private static class ShooterAligner extends DrivetrainGoalAligner {

        // holds the shooter because the shooter mode / distance changes
        private final Shooter shooter;

        public ShooterAligner(Shooter shooter) {
            super(0); // default distance of zero
            this.shooter = shooter;
        }

        public void init() {
            super.init(); // turns on the limelight
            this.distance = shooter.getMode().distance;
        }

    }

    public DrivetrainAutomaticAlign(Drivetrain drivetrain, Shooter shooter) {
        super(drivetrain, new ShooterAligner(shooter));
    }

}
