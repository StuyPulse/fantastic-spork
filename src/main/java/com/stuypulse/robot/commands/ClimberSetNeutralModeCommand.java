package com.stuypulse.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberSetNeutralModeCommand extends InstantCommand {
    
    Climber climber;
    IdleMode mode;

    public ClimberSetNeutralModeCommand(Climber climber, IdleMode mode) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setNeutralMode(mode);
    }

}