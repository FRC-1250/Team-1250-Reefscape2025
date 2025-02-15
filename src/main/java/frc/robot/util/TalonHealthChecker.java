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
                talon.getStickyFault_BootDuringEnable().getValue(),
                talon.getStickyFault_BridgeBrownout().getValue(),
                talon.getStickyFault_Hardware().getValue(),
                talon.getStickyFault_OverSupplyV().getValue(),
                talon.getStickyFault_RemoteSensorDataInvalid().getValue(),
                talon.getStickyFault_RemoteSensorReset().getValue(),
                talon.getStickyFault_Undervoltage().getValue(),
                talon.getStickyFault_UnstableSupplyV().getValue());
    }
}
