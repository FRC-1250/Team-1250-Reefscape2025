// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEndEffector extends SubsystemBase {
  /** Creates a new AlgaeEndEffector. */
  public AlgaeEndEffector() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX wrisTalonFX = new TalonFX(22);
  private TalonFX intakeTalonFX = new TalonFX(23);
  private DigitalInput IRSensor = new DigitalInput(5);
  private CANcoder absCaNcoder = new CANcoder(24);

  public enum Position {
    Home(0),
    Reef(0),
    Floor(0),
    Barge(0),
    Topofcoral(0);

    public final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }

    private PositionVoltage wristPositionControl = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage intakevolocitycontrol = new VelocityVoltage(0).withSlot(0);
    private PositionVoltage intakePositionControl = new PositionVoltage(0).withSlot(0);
  }
}
