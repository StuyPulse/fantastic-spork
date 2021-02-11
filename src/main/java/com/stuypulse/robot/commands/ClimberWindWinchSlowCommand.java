package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberWindWinchSlowCommand extends CommandBase {

    private final Climber climber;

    public ClimberWindWinchSlowCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.moveLiftDownSlow();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopClimber();
    }
}