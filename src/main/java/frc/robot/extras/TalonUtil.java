// moderately inspired by 254
package frc.robot.extras;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class TalonUtil {

  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, double timeoutSeconds, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (checkErrorAndRetry(() -> talon.getConfigurator().apply(config, timeoutSeconds))) {
        return true;
      }
    }
    DriverStation.reportError(
        "Failed to apply config for talon after " + numTries + " attempts", false);
    return false;
  }

  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, double timeoutMs) {
    boolean result = applyAndCheckConfiguration(talon, config, timeoutMs, 5);
    return result;
  }

  public enum StickyFault {
    BootDuringEnable,
    DeviceTemp,
    ForwardHardLimit,
    ForwardSoftLimit,
    Hardware,
    OverSupplyV,
    ProcTemp,
    ReverseHardLimit,
    ReverseSoftLimit,
    Undervoltage,
    UnstableSupplyV
  }

  public static void checkStickyFaults(String motorName, TalonFX talon) {
    boolean[] faults = new boolean[StickyFault.values().length];
    faults[0] = talon.getStickyFault_BootDuringEnable().getValue();
    faults[1] = talon.getStickyFault_DeviceTemp().getValue();
    faults[2] = talon.getStickyFault_ForwardHardLimit().getValue();
    faults[3] = talon.getStickyFault_ForwardSoftLimit().getValue();
    faults[4] = talon.getStickyFault_Hardware().getValue();
    faults[5] = talon.getStickyFault_OverSupplyV().getValue();
    faults[6] = talon.getStickyFault_ProcTemp().getValue();
    faults[7] = talon.getStickyFault_ReverseHardLimit().getValue();
    faults[8] = talon.getStickyFault_ReverseSoftLimit().getValue();
    faults[9] = talon.getStickyFault_Undervoltage().getValue();
    faults[10] = talon.getStickyFault_UnstableSupplyV().getValue();

    for (int i = 0; i < faults.length; i++) {
      if (faults[i]) {
        DriverStation.reportError(
            motorName + ": Talon Fault! " + StickyFault.values()[i].toString(), false);
      }
    }

    talon.clearStickyFaults();
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      DriverStation.reportError(
          "Failed to execute phoenix pro api call after " + numTries + " attempts", false);
      return false;
    }
    return true;
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
    return checkErrorAndRetry(function, 5);
  }
}
