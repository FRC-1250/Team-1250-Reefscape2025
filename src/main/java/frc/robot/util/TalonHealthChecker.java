package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonHealthChecker extends PhoenixHealthChecker {

    private final TalonFX talon;

    public TalonHealthChecker(TalonFX talon, String subsystemName) {
        super(talon, subsystemName);
        this.talon = talon;
    }

    @Override
    protected List<Boolean> faultsToBooleanList() {
        return Arrays.asList(
                talon.getFault_BootDuringEnable().getValue(),
                // talon.getStickyFault_BridgeBrownout().getValue(),
                // talon.getStickyFault_Hardware().getValue(),
                talon.getFault_OverSupplyV().getValue(),
                talon.getFault_RemoteSensorDataInvalid().getValue(),
                talon.getFault_RemoteSensorReset().getValue(),
                talon.getFault_Undervoltage().getValue(),
                talon.getFault_UnstableSupplyV().getValue());
    }
}
