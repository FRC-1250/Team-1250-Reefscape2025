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
                cancoder.getFault_BootDuringEnable().getValue(),
                cancoder.getFault_BadMagnet().getValue(),
                // cancoder.getStickyFault_Hardware().getValue(),
                cancoder.getFault_Undervoltage().getValue());
    }
}
