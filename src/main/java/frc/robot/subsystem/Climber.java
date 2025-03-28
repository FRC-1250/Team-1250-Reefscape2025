// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class Climber extends SubsystemBase {

  private TalonFX climber = new TalonFX(40);
  private TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);
  private final double requiredRotations = 244; // 235 ~= 13 inches of travel on the climber

  public Climber() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    climber.getConfigurator().apply(talonFXConfiguration);
    climber.setPosition(0);
    climber.getPosition().setUpdateFrequency(200);
   
    HealthMonitor.getInstance()
        .addComponent(getName(), "Winch", climber);
  }

  @Logged(name = "Torque current")
  public double getTorque() {
    return climber.getTorqueCurrent().getValueAsDouble();
  }

  public void setTorque(Current newCurrent) {
    climber.setControl(torqueControl.withOutput(newCurrent));
  }

  @Logged(name = "Position")
  public double getPosition() {
    return climber.getPosition().getValueAsDouble();
  }

  public void stopMotor() {
    climber.stopMotor();
  }

  public boolean hasPassedClimbThreshold() {
    return getPosition() >= requiredRotations;
  }

  @Override
  public void periodic() {
    
  }
}
