package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase {
  
  private final Pigeon2 pigeon = new Pigeon2(60);
  private double offset = 0;

  public Pigeon() {
    // Constructor
  }

  /**
   * Returns the accumulated gyro angle (Z-axis) with offset correction.
   * @return The corrected accumulated gyro value.
   */
  public double getAccumGyro() {
    return pigeon.getAccumGyroZ().getValueAsDouble() - offset;
  }

  /**
   * Returns the yaw angle in a range of [0, 360) degrees.
   * @return The corrected yaw angle.
   */
  public double getYaw() {
    double angle = getAccumGyro() % 360;
    return (angle >= 0) ? angle : angle + 360;
  }

  /**
   * Resets the gyro by adjusting the offset.
   */
  public void resetGyro() {
    offset += getAccumGyro();
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with gyro values
    SmartDashboard.putNumber("Gyro Yaw", getYaw());
    SmartDashboard.putNumber("Accumulated Gyro Z", pigeon.getAccumGyroZ().getValueAsDouble());
    SmartDashboard.putNumber("Corrected Rotation", (((0 - getYaw() + 180) % 360 + 360) % 360 - 180));
  }
}
