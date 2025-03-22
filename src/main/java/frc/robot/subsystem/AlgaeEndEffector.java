// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class AlgaeEndEffector extends SubsystemBase {

  public enum IntakeVelocity {
    STOP(0),
    SLOW_INTAKE(-50),
    INTAKE(-75),
    YOINK(-100),
    SLOW_RELEASE(50),
    RELEASE(75),
    YEET(100);

    public final double rotationsPerSecond;

    IntakeVelocity(double rotationsPerSecond) {
      this.rotationsPerSecond = rotationsPerSecond;
    }
  }

  public enum WristPosition {
    HOME(0.31),
    ALGAE_CONTAINMENT(.41),
    PROCESSOR(.59),
    L1(0.43),
    REEF(0.51),
    FLOOR(0.67),
    BARGE(0.31);

    public final double rotations;

    WristPosition(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX wristTalonFX = new TalonFX(22);
  private CANcoder wristAbsoluteEncoder = new CANcoder(24);
  private PositionVoltage wristPositionControl = new PositionVoltage(0).withSlot(0);
  private final double encoderOffset = 0.95888671875;

  private TalonFX intakeTalonFX = new TalonFX(23);
  private DigitalInput intakeAlgaeSensor = new DigitalInput(5);
  private VelocityVoltage intakeVelocityControl = new VelocityVoltage(0).withSlot(1);

  public AlgaeEndEffector() {
    CANcoderConfiguration wristAbsoluteEncoderConfiguration = new CANcoderConfiguration();
    // Set encoder to provide a value between 0 and 1
    wristAbsoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    wristAbsoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristAbsoluteEncoderConfiguration.MagnetSensor.MagnetOffset = encoderOffset;
    wristAbsoluteEncoder.getConfigurator().apply(wristAbsoluteEncoderConfiguration);
    wristAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(200);

    Slot0Configs wristPositionPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        .withKS(0.25)
        .withKP(25)
        .withKI(0)
        .withKD(0.01);

    TalonFXConfiguration wristTalonFXConfiguration = new TalonFXConfiguration();
    wristTalonFXConfiguration.Slot0 = wristPositionPIDConfigs;
    wristTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    wristTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.7;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.30;
    wristTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = wristAbsoluteEncoder.getDeviceID();
    wristTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristTalonFX.getConfigurator().apply(wristTalonFXConfiguration);

    Slot1Configs intakeVelocityPIDConfigs = new Slot1Configs()
        .withKS(0.1)
        .withKV(0.12)
        .withKP(0.11)
        .withKI(0)
        .withKD(0);

    TalonFXConfiguration intakeTalonFXConfiguration = new TalonFXConfiguration();
    intakeTalonFXConfiguration.Slot1 = intakeVelocityPIDConfigs;
    intakeTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    intakeTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeTalonFX.getConfigurator().apply(intakeTalonFXConfiguration);

    HealthMonitor.getInstance()
        .addComponent(getName(), "Wrist", wristTalonFX)
        .addComponent(getName(), "Intake", intakeTalonFX)
        .addComponent(getName(), "Wrist encoder", wristAbsoluteEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
   * Wrist
   */

  @Logged(name = "Wrist position")
  public double getWristPosition() {
    return wristAbsoluteEncoder.getPosition().getValueAsDouble();
  }

  private void setWristPosition(double rotations) {
    wristTalonFX.setControl(
        wristPositionControl
            .withPosition(rotations)
            .withFeedForward(Volts.of(0))
            .withEnableFOC(true));
  }

  public void setWristPosition(WristPosition postion) {
    setWristPosition(postion.rotations);
  }

  public void addRotationsToWristPosition(double rotationsToAdd) {
    setWristPosition(rotationsToAdd + getWristPosition());
  }

  public boolean isWristNearPosition(WristPosition position) {
    return isWristNearPosition(position.rotations);
  }

  public boolean isWristNearPosition(double rotations) {
    return MathUtil.isNear(rotations, getWristPosition(), 0.01);
  }

  /*
   * Intake
   */

  @Logged(name = "Intake velocity")
  public double getIntakeVelocity() {
    return intakeTalonFX.getVelocity().getValueAsDouble();
  }

  public void setIntakeVelocity(IntakeVelocity velocity) {
    intakeTalonFX.setControl(intakeVelocityControl.withVelocity(velocity.rotationsPerSecond));
  }
@Logged
  public boolean hasAlgae() {
    return !intakeAlgaeSensor.get();
  }
}
