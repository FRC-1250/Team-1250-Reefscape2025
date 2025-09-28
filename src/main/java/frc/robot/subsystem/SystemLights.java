// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SystemLights extends SubsystemBase {

    private final CANdle candle = new CANdle(30, "rio");
    private SolidColor allLEDs = new SolidColor(0, 7);

    public final static RGBWColor red = new RGBWColor(Color.kRed);
    public final static RGBWColor white = new RGBWColor(Color.kWhite);
    public final static RGBWColor green = new RGBWColor(Color.kGreen);
    public final static RGBWColor blue = new RGBWColor(Color.kBlue);
    public final static RGBWColor purple = new RGBWColor(Color.kPurple);
    public final static RGBWColor pink = new RGBWColor(Color.kPink);
    public final static RGBWColor kellyGreen = new RGBWColor(76, 187, 23);

    public SystemLights() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.LED.BrightnessScalar = 0.6;
        candleConfiguration.LED.StripType = StripTypeValue.RGB;
        candleConfiguration.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        candleConfiguration.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candleConfiguration.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
        candleConfiguration.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        candle.getConfigurator().apply(candleConfiguration);
    }

    public void setLEDs(RGBWColor color) {
        candle.setControl(allLEDs.withColor(color));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
