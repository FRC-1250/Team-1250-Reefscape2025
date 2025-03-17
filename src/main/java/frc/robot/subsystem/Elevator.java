// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthStatus;
import frc.robot.util.TalonHealthChecker;
import frc.robot.util.TunableTalonFX;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    STARTING_CONFIG(0),
    SENSOR(1.3),
    HOME(8),
    LOW_ALGAE(32.1),
    HIGH_ALGAE(46.7),
    BARGE(60),
    PEAK(61.5);

    public final double rotations;

    ElevatorPosition(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX leftMotor = new TalonFX(30);
  private TalonFX rightMotor = new TalonFX(31);
  private DigitalInput coralSensor = new DigitalInput(0);
  private DigitalInput homeSensor = new DigitalInput(1);
  private MotionMagicVoltage motionMagicPostionControl = new MotionMagicVoltage(0).withEnableFOC(false);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(false);
  private boolean homeFound = false;
  private boolean previousHomeSensor = isAtHome();

  private final boolean tuningModeEnabled = false;
  private final boolean healthCheckEnabled = true;
  private TunableTalonFX tunableTalonFX;
  private TalonHealthChecker leftMotorCheck;
  private TalonHealthChecker rightMotorCheck;
  private HealthStatus healthStatus = HealthStatus.IS_OK;

  public Elevator() {
    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.3)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withKS(0.2)
        .withKV(0.25)
        .withKP(7)
        .withKI(0)
        .withKD(0);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(75))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(175))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = ElevatorPosition.PEAK.rotations;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = ElevatorPosition.HOME.rotations;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 25;

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.DutyCycleNeutralDeadband = 0.05;

    VoltageConfigs voltageConfigs = new VoltageConfigs();
    voltageConfigs.PeakForwardVoltage = 12;
    voltageConfigs.PeakReverseVoltage = -4;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;
    talonFXConfiguration.Slot0 = positionPIDConfigs;
    talonFXConfiguration.MotionMagic = motionMagicConfigs;
    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;
    talonFXConfiguration.Voltage = voltageConfigs;

    leftMotor.getConfigurator().apply(talonFXConfiguration);
    rightMotor.getConfigurator().apply(talonFXConfiguration);

    BaseStatusSignal.setUpdateFrequencyForAll(200,
        leftMotor.getPosition(),
        rightMotor.getPosition());

    if (tuningModeEnabled) {
      tunableTalonFX = new TunableTalonFX(getName(), "Left + right motors", SlotConfigs.from(positionPIDConfigs),
          leftMotor,
          rightMotor);
    }

    if (healthCheckEnabled) {
      leftMotorCheck = new TalonHealthChecker(leftMotor, getName());
      rightMotorCheck = new TalonHealthChecker(rightMotor, getName());
    }
    cmdHandleSensorTransition().schedule();
  }

  @Override
  public void periodic() {
    // TODO: Verify if notifier command works better
    // detectSensorTransition();

    if (tuningModeEnabled) {
      tunableTalonFX.updateValuesFromSmartNT();
    }

    if (healthCheckEnabled) {
      if (!leftMotorCheck.isDeviceHealthy() |
          !rightMotorCheck.isDeviceHealthy()) {
        healthStatus = HealthStatus.ERROR;
      } else {
        healthStatus = HealthStatus.IS_OK;
      }
    }
  }

  public Command cmdHandleSensorTransition() {
    return new NotifierCommand(() -> detectSensorTransition(), 0.002)
        .until(() -> homeFound)
        .unless(() -> homeFound)
        .withName("Elevator find home")
        .ignoringDisable(true);
  }

  private void detectSensorTransition() {
    var isAtHome = isAtHome();
    if (previousHomeSensor != isAtHome) {
      homeFound = true;
      resetMotorPositionToPosition(ElevatorPosition.SENSOR.rotations);
    }
    previousHomeSensor = isAtHome;
  }

  @Logged(name = "Home found")
  public boolean getHomeFound() {
    return homeFound;
  }

  @Logged(name = "Has coral")
  public boolean hasCoralInChute() {
    return !coralSensor.get();
  }

  @Logged(name = "Right rotations")
  public double getRightMotorPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  @Logged(name = "Left rotations")
  public double getLeftMotorPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  public double getLeftMotorVelocity() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public double getRightMotorVelocity() {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  @Logged(name = "Health status")
  public HealthStatus getHealthStatus() {
    return healthStatus;
  }

  public boolean isAbovePosition(ElevatorPosition position) {
    var pos = position.rotations - 2; // Apply some buffer
    return pos <= getLeftMotorPosition() || pos <= getRightMotorPosition();
  }

  @Logged(name = "Home")
  public boolean isAtHome() {
    return !homeSensor.get();
  }

  public void setPosition(double position) {
    motionMagicPostionControl.Position = position;
    leftMotor.setControl(motionMagicPostionControl);
    rightMotor.setControl(motionMagicPostionControl);
  }

  public void resetMotorPositionToPosition(double rotations) {
    leftMotor.setPosition(rotations);
    rightMotor.setPosition(rotations);
  }

  public void addRotationsToElevatorPosition(double rotationsToAdd) {
    setPosition(rotationsToAdd + getLeftMotorPosition());
  }

  public Command cmdResetPos() {
    return Commands.runOnce(() -> resetMotorPositionToPosition(ElevatorPosition.HOME.rotations), this).ignoringDisable(true);
  }

  public boolean isNearPositionAndTolerance(double position, double tolerance) {
    return MathUtil.isNear(position, getLeftMotorPosition(), tolerance)
        || MathUtil.isNear(position, getRightMotorPosition(), tolerance);
  }

}
