/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.stuypulse.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {

    /*********************************************************************************************
     * Shooter Motor Ports
     *********************************************************************************************/
    int LEFT_SHOOTER_MOTOR_PORT = -1;
    int RIGHT_SHOOTER_MOTOR_PORT = -1;
    int MIDDLE_SHOOTER_MOTOR_PORT = -1;
    
    /*********************************************************************************************
     * Feeder Motor Port
     *********************************************************************************************/
    int FEEDER_MOTOR_PORT = -1;

    /*********************************************************************************************
     * Hood Solenoid Port
     *********************************************************************************************/
    int HOOD_SOLENOID_PORT = -1;

    /*********************************************************************************************
     * PID 
     *********************************************************************************************/
    double SHOOTER_SHOOT_KP = 0;
    double SHOOTER_SHOOT_KI = 0;
    double SHOOTER_SHOOT_KD = 0;

    /*********************************************************************************************
     * Shooter Constants
     *********************************************************************************************/
    double SHOOTER_WHEEL_DIAMTER = 4;
    double SHOOTER_WHEEL_CIRCUMFERENCE = Math.PI * SHOOTER_WHEEL_DIAMTER;
    double SHOOTER_VELOCITY_RAW_MULTIPLIER = SHOOTER_WHEEL_CIRCUMFERENCE / 60;
    double SHOOTER_VELOCITY_EMPIRICAL_MULTIPLER = SHOOTER_VELOCITY_RAW_MULTIPLIER; // TODO

    double SHOOTER_MAX_RPM = 16500;
    double SHOOT_FROM_INITATION_LINE_RPM = 3900;
    double SHOOT_FROM_TRENCH_RPM = 4900;
    double SHOOT_FROM_FAR_RPM = 5500;
}