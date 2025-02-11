// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Add your docs here. */
public class TalonHealthChecker {

    private final TalonFX talon;
    private final Alert alert;
    private boolean deviceIsHealthy = true;
    private Timer timer = new Timer();
    private double minTimeBetweenCheckups = 1;

    public TalonHealthChecker(TalonFX talon, String subsystemName) {
        this.talon = talon;
        alert = new Alert(String.format("%s - %d, check engine", subsystemName, talon.getDeviceID()), AlertType.kError);
        timer.start();
    }

    public void checkUp() {
        if (timer.hasElapsed(minTimeBetweenCheckups)) {
            deviceIsHealthy = true;

            if (!(talon.isAlive() && talon.isConnected())) {
                deviceIsHealthy = false;

            }

            if (areBooleanStatusSignalsHealthy(faultsToBooleanList(talon))) {
                deviceIsHealthy = false;
            }

            alert.set(deviceIsHealthy);
            timer.reset();
        }
    }

    private boolean areBooleanStatusSignalsHealthy(List<Boolean> statusSignalValues) {
        for (boolean statusSignalValue : statusSignalValues) {
            if (statusSignalValue) {
                return false;
            }
        }
        return true;
    }

    private List<Boolean> faultsToBooleanList(TalonFX talon) {
        return Arrays.asList(talon.getStickyFault_BootDuringEnable().getValue(),
                talon.getStickyFault_BridgeBrownout().getValue(),
                talon.getStickyFault_Hardware().getValue(),
                talon.getStickyFault_OverSupplyV().getValue(),
                talon.getStickyFault_RemoteSensorDataInvalid().getValue(),
                talon.getStickyFault_RemoteSensorReset().getValue(),
                talon.getStickyFault_Undervoltage().getValue(),
                talon.getStickyFault_UnstableSupplyV().getValue());
    }
}
