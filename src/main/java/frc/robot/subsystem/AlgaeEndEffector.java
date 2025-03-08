// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEndEffector extends SubsystemBase {

  public enum IntakeVelocity {
    SLOW_INTAKE(-10),
    INTAKE(-20),
    YOINK(-20),
    SLOW_RELEASE(10),
    RELEASE(20),
    YEET(20);

    public final double rotationsPerSecond;

    IntakeVelocity(double rotationsPerSecond) {
      this.rotationsPerSecond = rotationsPerSecond;
    }
  }

  public enum WristPosition {
    HOME(0),
    REEF(0),
    FLOOR(0),
    BARGE(0),
    TOP_OF_CORAL(0);

    public final double rotations;

    WristPosition(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX wristTalonFX = new TalonFX(22);
  private CANcoder wristAbsoluteEncoder = new CANcoder(24);
  private PositionVoltage wristPositionControl = new PositionVoltage(0).withSlot(0);
  private DutyCycleOut wristDutyCycle = new DutyCycleOut(0);

  private TalonFX intakeTalonFX = new TalonFX(23);
  private DigitalInput intakeAlgaeSensor = new DigitalInput(5);
  private PositionVoltage intakePositionControl = new PositionVoltage(0).withSlot(0);
  private VelocityVoltage intakeVelocityControl = new VelocityVoltage(0).withSlot(1);
  private DutyCycleOut intakeDutyCycle = new DutyCycleOut(0);

  public AlgaeEndEffector() {
    CANcoderConfiguration wristAbsoluteEncoderConfiguration = new CANcoderConfiguration();
    // Set encoder to provide a value between 0 and 1
    wristAbsoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    wristAbsoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    wristAbsoluteEncoderConfiguration.MagnetSensor.MagnetOffset = 0;
    wristAbsoluteEncoder.getConfigurator().apply(wristAbsoluteEncoderConfiguration);
    wristAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(200);

    Slot0Configs wristPositionPIDConfigs = new Slot0Configs()
        .withKG(0)
        .withKS(0)
        .withKP(1)
        .withKI(0)
        .withKD(0)
        .withKV(0);

    TalonFXConfiguration wristTalonFXConfiguration = new TalonFXConfiguration();
    wristTalonFXConfiguration.Slot0 = wristPositionPIDConfigs;
    wristTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
    wristTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    wristTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    wristTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = wristAbsoluteEncoder.getDeviceID();
    wristTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristTalonFXConfiguration.Feedback.RotorToSensorRatio = 1;
    wristTalonFX.getConfigurator().apply(wristTalonFXConfiguration);

    Slot0Configs intakePositionPIDConfigs = new Slot0Configs()
        .withKG(0)
        .withKS(0)
        .withKP(1)
        .withKI(0)
        .withKD(0)
        .withKV(0);

    Slot1Configs intakeVelocityPIDConfigs = new Slot1Configs()
        .withKG(0)
        .withKS(0)
        .withKP(1)
        .withKI(0)
        .withKD(0)
        .withKV(0);

    TalonFXConfiguration intakeTalonFXConfiguration = new TalonFXConfiguration();
    intakeTalonFXConfiguration.Slot0 = intakePositionPIDConfigs;
    intakeTalonFXConfiguration.Slot1 = intakeVelocityPIDConfigs;
    intakeTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
    intakeTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeTalonFX.getConfigurator().apply(intakeTalonFXConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
   * Wrist
   */
  public void stopWrist() {
    wristTalonFX.stopMotor();
  }

  public void setWristDutyCycle(double dutyCycleOutputPercent) {
    wristTalonFX.setControl(wristDutyCycle.withOutput(dutyCycleOutputPercent));
  }

  public double getWristPosition() {
    return wristAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }

  private void setWristPosition(double rotations) {
    wristTalonFX.setControl(wristPositionControl.withPosition(rotations));
  }

  public void setWristPosition(WristPosition postion) {
    setWristPosition(postion.rotations);
  }

  public void addRotationsToWristPosition(double rotationsToAdd) {
    setWristPosition(rotationsToAdd + getWristPosition());
  }

  public boolean isWristNearPosition(WristPosition position) {
    return MathUtil.isNear(position.rotations, getWristPosition(), 0.01);
  }

  /*
   * Intake
   */
  public void stopIntake() {
    intakeTalonFX.stopMotor();
  }

  public void setIntakeDutyCycleOut(double dutyCycleOutputPercent) {
    intakeTalonFX.setControl(intakeDutyCycle.withOutput(dutyCycleOutputPercent));
  }

  public void setIntakeVelocity(IntakeVelocity velocity) {
    intakeTalonFX.setControl(intakeVelocityControl.withVelocity(velocity.rotationsPerSecond));
  }

  private double getIntakePosition() {
    return intakeTalonFX.getPosition().getValueAsDouble();
  }

  public void holdPosition() {
    intakeTalonFX.setControl(intakePositionControl.withPosition(getIntakePosition()));
  }

  public boolean hasAlgae() {
    return !intakeAlgaeSensor.get();
  }
}
