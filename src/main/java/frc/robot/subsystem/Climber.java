// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthStatus;
import frc.robot.util.TalonHealthChecker;

public class Climber extends SubsystemBase {

  private TalonFX climber = new TalonFX(40);
  private final boolean healthCheckEnabled = true;
  private HealthStatus healthStatus = HealthStatus.IS_OK;
  private TalonHealthChecker climberCheck;
  private TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);

  public Climber() {
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 25;

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    climber.getConfigurator().apply(talonFXConfiguration);

    if (healthCheckEnabled) {
      climberCheck = new TalonHealthChecker(climber, getName());
    }
  }

  public Command cmdSetTorque(Current newCurrent) {
    return Commands.runEnd(
        () -> setTorque(newCurrent),
        () -> climber.stopMotor(), this)
        .withName(String.format("Climber set torque - %f", newCurrent.magnitude()));
  }

  public HealthStatus getHealthStatus() {
    return healthStatus;
  }

  @Logged(name = "Climber torque current")
  public double getTorque() {
    return climber.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    if (healthCheckEnabled) {
      if (!climberCheck.isDeviceHealthy()) {
        healthStatus = HealthStatus.ERROR;
      } else {
        healthStatus = HealthStatus.IS_OK;
      }
    }
  }

  private void setTorque(Current newCurrent) {
    climber.setControl(torqueControl.withOutput(newCurrent));
  }
}
