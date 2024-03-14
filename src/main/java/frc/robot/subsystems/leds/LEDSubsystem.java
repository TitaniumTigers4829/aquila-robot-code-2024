package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;


public class LEDSubsystem extends SubsystemBase {

  private Spark ledSpark;

  private LEDProcess process;

  private boolean isRed;

  /** Creates a new LEDSubsystemImpl with the port in LEDConstants. */
  public LEDSubsystem() {
    ledSpark = new Spark(LEDConstants.LED_PORT);
    setProcess(LEDProcess.OFF);
    isRed = DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == Alliance.Red);
  }
  
  /**
   * sets the process of the leds
   * @param process the process to set
   */
  public void setProcess(LEDProcess process) {
    this.process = process;
  }

  /**
   * gets the spark (color) value to set from the process
   * @param pr the process
   * @return the spark value from the process
   */
  private double getSparkFromProcess(LEDProcess pr) {
    switch (pr) {
      case DEFAULT:
        return defaultColor();
      case ALLIANCE_COLOR:
        return allianceColor();
      default:
        return pr.sparkValue;
    }
  }

  /**
   * the led alliance color based on what alliance we are on
   * @return the led color (red vs blue).
   */
  private double allianceColor() {
    if (isRed) {
      return LEDProcess.RED_ALLIANCE.sparkValue;
    } else {
      return LEDProcess.BLUE_ALLIANCE.sparkValue;
    }
  }

  /**
   * sets the default color of the leds.
   * @return auto led value it robot is in auto, otherwise alliance color
   */
  private double defaultColor() {
    if (DriverStation.isAutonomous()) {
      return LEDProcess.AUTONOMOUS.sparkValue;
    } else {
      return allianceColor();
    }
  }

  /**
   * turns off the leds
   */
  public void off() {
    process = LEDProcess.OFF;
  }

  @Override
  public void periodic() {
    if (process == LEDProcess.ALLIANCE_COLOR) {
      ledSpark.set(allianceColor());
    } else {
      ledSpark.set(getSparkFromProcess(process));
    }
  }
}