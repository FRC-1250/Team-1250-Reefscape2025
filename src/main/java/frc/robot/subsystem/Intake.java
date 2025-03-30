// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class Intake extends SubsystemBase {
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

    private TalonFX intakeTalonFX = new TalonFX(23);
    private DigitalInput intakeAlgaeSensor = new DigitalInput(5);
    private VelocityVoltage intakeVelocityControl = new VelocityVoltage(0).withSlot(1);

    public Intake() {
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
                .addComponent(getName(), "Intake", intakeTalonFX);
    }

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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
