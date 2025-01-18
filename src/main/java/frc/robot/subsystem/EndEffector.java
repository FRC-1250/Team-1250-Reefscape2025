// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  private TalonFX end1 = new TalonFX(21);
  private TalonFX end2 = new TalonFX(20);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  /** Creates a new EndEffector. */
  public EndEffector() {
    Slot0Configs endConfigs = new Slot0Configs()
    .withKG(0)
    .withKS(0)
    .withKP(0)
    .withKI(0)
    .withKD(0)
    .withKV(0);

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.Slot0 = endConfigs;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguration.Voltage.PeakForwardVoltage = 8;
    talonFXConfiguration.Voltage.PeakReverseVoltage = -4;

    end1.getConfigurator().apply(talonFXConfiguration);
    end2.getConfigurator().apply(talonFXConfiguration);

  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command cmdSetDutyCycleOut() {
    return Commands.none();
  }

  public double getSetDutyCycleOut() {
    return 0;
  }

  public Command cmdSetPostion() {
    return Commands.none();
  }

  public boolean cmdIsNearForwardLimit() {
    return false;
  }
  
  public boolean cmdIsNearReverseLimit() {
    return false;
  }

  public double cmdGetPositon() {
    return 0;
  }


}
