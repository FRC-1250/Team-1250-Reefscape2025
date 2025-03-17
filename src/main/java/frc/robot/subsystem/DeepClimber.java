// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class DeepClimber extends SubsystemBase {

  public enum DeepClimberPhase {
    PREP(10),
    CLIMB(20);

    public final double rotations;

    DeepClimberPhase(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX deepClimber = new TalonFX(41);
  private MotionMagicTorqueCurrentFOC motionMagicTorqueControl = new MotionMagicTorqueCurrentFOC(0);

  /** Creates a new DeepClimb. */
  public DeepClimber() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 50;
    motionMagicConfigs.MotionMagicAcceleration = 150;
    motionMagicConfigs.MotionMagicJerk = 0;

    Slot0Configs positionConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0.0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withKS(0.2)
        .withKV(0.25)
        .withKP(1)
        .withKI(0)
        .withKD(0.0);

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotorOutput = motorOutputConfigs;
    talonFXConfiguration.MotionMagic = motionMagicConfigs;
    talonFXConfiguration.Slot0 = positionConfigs;

    deepClimber.getConfigurator().apply(talonFXConfiguration);
    deepClimber.setPosition(0);
    deepClimber.getPosition().setUpdateFrequency(200);

    HealthMonitor.getInstance().addComponent(getName(), "Winch", deepClimber);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    deepClimber.stopMotor();
  }

  public void setPosition(DeepClimberPhase deepClimberPhase, Current feedforward) {
    deepClimber.setControl(motionMagicTorqueControl
        .withPosition(deepClimberPhase.rotations)
        .withFeedForward(feedforward)
        .withSlot(0));
  }

   public boolean isNearPositionAndTolerance(double position, double tolerance) {
    return MathUtil.isNear(position, getRotations(), tolerance);
  }

  @Logged(name = "Position")
  public double getRotations() {
    return deepClimber.getPosition().getValueAsDouble();
  }

  @Logged(name = "Amps")
  public double getCurrent() {
    return deepClimber.getStatorCurrent().getValueAsDouble();
  }

}