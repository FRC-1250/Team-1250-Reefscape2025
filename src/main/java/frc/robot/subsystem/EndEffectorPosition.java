// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

/*
 * Servo position and offsets for rotating the EndEffector
 * Configure the true center to automatically adjust all other positions
 */
public class EndEffectorPosition {
    private final static double TURE_CENTER = 0.5;
    private final static double CORAL_BRANCH_SERVO_OFFSET = 0.15;
    private final static double NINTY_DEGREE_OFFSET = 0.3;

    public final static double CENTER = TURE_CENTER;
    public final static double CENTER_LEFT = TURE_CENTER - CORAL_BRANCH_SERVO_OFFSET;
    public final static double LEFT = TURE_CENTER - NINTY_DEGREE_OFFSET;
    public final static double CENTER_RIGHT = TURE_CENTER + CORAL_BRANCH_SERVO_OFFSET;
    public final static double RIGHT = TURE_CENTER + NINTY_DEGREE_OFFSET;
}
