package frc.robot.subsystems.ScoringMecanisms;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import frc.robot.Calibrations;
import frc.robot.Calibrations.ElevatorCalibrations;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_motor;
  private final TalonFXConfiguration m_config;
  private final DynamicMotionMagicTorqueCurrentFOC m_request;
  public ElevatorSubsystem() {
    m_motor = new TalonFX(20); // Adjust CAN ID as necessary
    m_config = new TalonFXConfiguration();
    

    m_request = new DynamicMotionMagicTorqueCurrentFOC(
            0, 
            ElevatorCalibrations.kMaxSpeedMotionMagic, 
            ElevatorCalibrations.kMaxAccelerationMotionMagic, 
            0);
    m_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    m_config.Slot0.kG = ElevatorCalibrations.kG;
        m_config.Slot0.kS = ElevatorCalibrations.kS;
        m_config.Slot0.kV = ElevatorCalibrations.kV;
        m_config.Slot0.kA = ElevatorCalibrations.kA;
        m_config.Slot0.kP = ElevatorCalibrations.kDownP;
        m_config.Slot0.kD = ElevatorCalibrations.kD;

        // Slot gains for going up
        m_config.Slot1.kG = ElevatorCalibrations.kG;
        m_config.Slot1.kS = ElevatorCalibrations.kS;
        m_config.Slot1.kV = ElevatorCalibrations.kV;
        m_config.Slot1.kA = ElevatorCalibrations.kA;
        m_config.Slot1.kP = ElevatorCalibrations.kUpP;
        m_config.Slot1.kD = ElevatorCalibrations.kD;

        m_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorCalibrations.kMaxSpeedMotionMagic;
        m_config.MotionMagic.MotionMagicAcceleration = ElevatorCalibrations.kMaxAccelerationMotionMagic;

        m_config.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorCalibrations.kMaxCurrentPerMotor;
        m_config.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorCalibrations.kMaxCurrentPerMotor;

        m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorCalibrations.kForwardSoftLimitThreshold;

        m_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_motor.getConfigurator().apply(m_config); 
  }
  public void updateSetpoint(double newSetpoint) {
    if (getPosition() >= newSetpoint) {
        m_motor.setControl(m_request.withPosition(newSetpoint)
                            .withVelocity(ElevatorCalibrations.kMaxSpeedMotionMagic)
                            .withSlot(0));
    } else {
        m_motor.setControl(m_request.withPosition(newSetpoint)
                            .withVelocity(ElevatorCalibrations.kMaxSpeedMotionMagic)
                            .withSlot(1));
    }
    

  }
  public boolean atTarget() {
    return Math.abs(getPosition() - getSetpoint()) < ElevatorCalibrations.kDefaultTolerance;
  }
/**
     * Return the position of the elevator (in).
     *
     * @return Position of the elevator (in)
     */
    public double getPosition() {
      return m_motor.getPosition().getValueAsDouble();
  }

  /**
   * Return elevator setpoint in inches.
   *
   * @return setpoint as a number in inches
   */
  public double getSetpoint() {
      return m_request.Position;
  }
public boolean isWithinTolerance(double tolerance) {
    // System.out.println(Math.abs(getPosition() - getSetpoint()));
    return Math.abs(getPosition() - getSetpoint()) < tolerance;
  }
  public void setPosition(){
    m_motor.setPosition(0);
  }
  public double getEncoderPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }
  public void setSpeed(double speed) {
      m_motor.set(speed);
  }
  public void setElevatorZeroDutyCycle() {
    m_motor.set(0);
  }

  public void setElevatorOpenLoopDutyCycle(double dutyCycle) {
      m_motor.set(dutyCycle);
  }

  public void setZeroPosition() {
      m_motor.setPosition(0);
  }
  public void setElevatorHeight(double low, double high, double speed){
      if (getEncoderPosition() > -high) {
        setSpeed(-speed);
      } else if (getEncoderPosition() < -low){
        setSpeed(speed);
      } else {
        setSpeed(0);
      }
  }
  public void highReef() {
    setElevatorHeight(160, 152.5, 1);
  }
  public void L3Reef() {
    setElevatorHeight(40, 29.2, .7);
  }
  public void Pickup() {
    setElevatorHeight(56.5, 45, 1);
  }
  public void bottomPosition() {
    setElevatorHeight(8, 0, 1);
  }
  public void L2Reef() {
    setElevatorHeight(69, 62, .9);
  }
  public void stop() {
    setSpeed(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", getEncoderPosition());
  }
}
