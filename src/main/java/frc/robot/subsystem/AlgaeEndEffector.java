// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEndEffector extends SubsystemBase {
  public enum Position {
    HOME(0),
    REEF(0),
    FLOOR(0),
    BARGE(0),
    TOP_OF_CORAL(0);

    public final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX wristTalonFX = new TalonFX(22);
  private CANcoder absoluteEncoder = new CANcoder(24);
  private PositionVoltage wristPositionControl = new PositionVoltage(0).withSlot(0);

  private TalonFX intakeTalonFX = new TalonFX(23);
  private DigitalInput algaeSensor = new DigitalInput(5);
  private VelocityVoltage intakevolocitycontrol = new VelocityVoltage(0).withSlot(0);
  private PositionVoltage intakePositionControl = new PositionVoltage(0).withSlot(0);

  public AlgaeEndEffector() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
