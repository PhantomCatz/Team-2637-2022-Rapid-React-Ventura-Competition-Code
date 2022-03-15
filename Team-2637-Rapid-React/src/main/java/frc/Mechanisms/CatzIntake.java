package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CatzIntake
{
    public WPI_TalonSRX intakeRoller;

    private final int TALON_MC_ID            = 30; 
    private final int INTAKE_DEPLOY_PCM_PORT  = 6;
    private final int INTAKE_RETRACT_PCM_PORT = 7;

    private final double INTAKE_ROLLER_MOTOR_POWER      = 0.8;
    private final double OUTTAKE_ROLLER_MOTOR_POWER     = 1.0;
    public final double AUTON_INTAKE_ROLLER_MOTOR_POWER = 0.8;
    private final double INTAKE_MOTOR_POWER_OFF         = 0.0;

    public final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
    private DoubleSolenoid intakeDeploySolenoid;

    public CatzIntake()
    {
        intakeRoller         = new WPI_TalonSRX(TALON_MC_ID);
        intakeDeploySolenoid = new DoubleSolenoid(PCM_TYPE, INTAKE_DEPLOY_PCM_PORT, INTAKE_RETRACT_PCM_PORT); 
        intakeRoller.configFactoryDefault();
        stowIntake();
    }
        
//-----------------------------------------------------------Roller----------------------------------------------------------

    public void intakeRollerIn()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_MOTOR_POWER);
    }

    public void intakeRollerInAuton()
    {
        intakeRoller.set(ControlMode.PercentOutput, AUTON_INTAKE_ROLLER_MOTOR_POWER);
    }

    public void intakeRollerOut()
    {
        intakeRoller.set(ControlMode.PercentOutput, -OUTTAKE_ROLLER_MOTOR_POWER);
    }

    public void intakeRollerOff()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);
    }

//-----------------------------------------------------------Deploy/Stow--------------------------------------------------------

    public void deployIntake() 
    {
        intakeDeploySolenoid.set(Value.kForward);
    }

    public void stowIntake() 
    {
        intakeDeploySolenoid.set(Value.kReverse);
    }

    public Value getIntakeState()
    {
        return intakeDeploySolenoid.get();
    }
}