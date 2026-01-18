package frc.robot.subsystems.appa;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class Appa {
  private final SparkMax m_conveyor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_wheel1 = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax m_wheel2 = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);

  public Appa() {
    m_conveyor.set(0.5);
    m_wheel1.set(0.5);
    m_wheel2.set(0.5);
  }

  public void stop() {
    m_conveyor.stopMotor();
    m_wheel1.stopMotor();
    m_wheel2.stopMotor();
  }
}
