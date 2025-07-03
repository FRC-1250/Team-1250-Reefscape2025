// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HealthMonitor;

public class Wrist extends SubsystemBase {
    public enum WristPosition {
        HOME(0.36),
        ALGAE_CONTAINMENT(0.26),
        PROCESSOR(0.08),
        L1(0.24),
        REEF(0.16),
        FLOOR(0),
        BARGE(0.36);

        public final double rotations;

        WristPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private TalonFX wristTalonFX = new TalonFX(22);
    private CANcoder wristAbsoluteEncoder = new CANcoder(24);
    private PositionVoltage wristPositionControl = new PositionVoltage(0).withSlot(0);
    private final double encoderOffset = 0.95888671875;

    public Wrist() {
        CANcoderConfiguration wristAbsoluteEncoderConfiguration = new CANcoderConfiguration();
        // Set encoder to provide a value between 0 and 1
        wristAbsoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        wristAbsoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
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
        wristTalonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        wristTalonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.35;
        wristTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        wristTalonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        wristTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = wristAbsoluteEncoder.getDeviceID();
        wristTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        wristTalonFXConfiguration.Feedback.RotorToSensorRatio = 2.0;
        wristTalonFX.getConfigurator().apply(wristTalonFXConfiguration);

        HealthMonitor.getInstance()
                .addComponent(getName(), "Wrist", wristTalonFX)
                .addComponent(getName(), "Wrist encoder", wristAbsoluteEncoder);
    }

    @Logged(name = "Wrist position")
    public double getWristPosition() {
        return wristAbsoluteEncoder.getPosition().getValueAsDouble();
    }

    @Logged(name = "Wrist position")
    public double getWristMotorPosition() {
        return wristTalonFX.getRotorPosition().getValueAsDouble();
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
