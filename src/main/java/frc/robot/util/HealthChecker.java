// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class HealthChecker {
    private static List<String> issues = new ArrayList<String>();

    public static boolean isTalonHealthy(TalonFX talon) {
        boolean healthy = true;
        issues.clear();

        if (!(talon.isAlive() && talon.isConnected())) {
            healthy = false;
            issues.add(String.format("Device %d on the %s may be offline or not responsive", talon.getDeviceID(),
                    talon.getNetwork()));
        }

        if (talon.getStickyFaultField().getValue() > 0 && healthy) {
            healthy = false;
            checkBooleanStatusSignal(talon.getStickyFault_BootDuringEnable());
            checkBooleanStatusSignal(talon.getStickyFault_BridgeBrownout());
            checkBooleanStatusSignal(talon.getStickyFault_Hardware());
            checkBooleanStatusSignal(talon.getStickyFault_OverSupplyV());
            checkBooleanStatusSignal(talon.getStickyFault_RemoteSensorDataInvalid());
            checkBooleanStatusSignal(talon.getStickyFault_RemoteSensorReset());
            checkBooleanStatusSignal(talon.getStickyFault_Undervoltage());
            checkBooleanStatusSignal(talon.getStickyFault_UnstableSupplyV());
        }
        return healthy;
    }

    private static void checkBooleanStatusSignal(StatusSignal<Boolean> status) {
        if (status.getValue()) {
            issues.add(String.format("Device %d has fault: %s", status.getName()));
        }
    }

    public List<String> getIssues() {
        return issues;
    }
}
