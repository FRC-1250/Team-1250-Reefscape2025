package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;

public class CANcoderHealthChecker extends PhoenixHealthChecker {

    private final CANcoder cancoder;

    public CANcoderHealthChecker(CANcoder cancoder, String subsystemName) {
        super(cancoder, subsystemName);
        this.cancoder = cancoder;
    }

    @Override
    protected List<Boolean> faultsToBooleanList() {
        return Arrays.asList(
                cancoder.getStickyFault_BootDuringEnable().getValue(),
                cancoder.getStickyFault_BadMagnet().getValue(),
                // cancoder.getStickyFault_Hardware().getValue(),
                cancoder.getStickyFault_Undervoltage().getValue());
    }
}
