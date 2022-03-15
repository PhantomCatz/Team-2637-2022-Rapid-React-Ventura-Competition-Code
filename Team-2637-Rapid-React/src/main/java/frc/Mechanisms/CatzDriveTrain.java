package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CatzDriveTrain
{

    /*---------------------------------------------------------------------------------------------------------------------------------
    *  Motor Definitions
    *--------------------------------------------------------------------------------------------------------------------------------*/
    public  final int     DRVTRAIN_LT_FRNT_MC_CAN_ID    = 1;
    public  final int     DRVTRAIN_LT_BACK_MC_CAN_ID    = 2;
    public  final int     DRVTRAIN_RT_FRNT_MC_CAN_ID    = 3;
    public  final int     DRVTRAIN_RT_BACK_MC_CAN_ID    = 4;

    public  final int     DRVTRAIN_LT_FRNT_MC_PDP_PORT   = 14;
    public  final int     DRVTRAIN_LT_BACK_MC_PDP_PORT   = 15;
    public  final int     DRVTRAIN_RT_FRNT_MC_PDP_PORT   = 0;  
    public  final int     DRVTRAIN_RT_BACK_MC_PDP_PORT   = 1; 

    private final int     CURRENT_LIMIT_AMPS            = 60;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 80;
    private final int     CURRENT_LIMIT_TIMEOUT_SECONDS = 5;

    private final boolean ENABLE_CURRENT_LIMIT          = true;

    public WPI_TalonFX drvTrainMtrCtrlLTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlLTBack;
    public WPI_TalonFX drvTrainMtrCtrlRTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlRTBack;

    private SupplyCurrentLimitConfiguration drvTrainCurrentLimit;

    /*----------------------------------------------------------------------------------------------
    *  Drive Definitions
    *---------------------------------------------------------------------------------------------*/
    public final int DRVTRAIN_LT = 0;
    public final int DRVTRAIN_RT = 1;
   
    private  DifferentialDrive    drvTrainDifferentialDrive;

    private  MotorControllerGroup drvTrainLT;
    private  MotorControllerGroup drvTrainRT;
    
    /*----------------------------------------------------------------------------------------------
    *  Gearbox & Encoder Definitions
    *---------------------------------------------------------------------------------------------*/
    private final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;

    private final int DRVTRAIN_GEARSHIFT_LO_PCM_PORT = 4;
    private final int DRVTRAIN_GEARSHIFT_HI_PCM_PORT = 3;

    public  final int DRVTRAIN_GEAR_LO  = 0;
    public  final int DRVTRAIN_GEAR_HI  = 1;

    private final double LOW_GEAR_RATIO  =  25.0 /  3.0; //TBD do calculations, method to determine gear and apply conversion factor
    private final double HIGH_GEAR_RATIO =  857.0 / 50.0;

    private final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    private final double DRVTRAIN_WHEEL_RADIUS                    = 2.8;
    private final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (2.0 * Math.PI * DRVTRAIN_WHEEL_RADIUS);

    public final double DRVTRAIN_ENC_COUNTS_TO_INCHES_LO = (1/LOW_GEAR_RATIO ) * DRVTRAIN_WHEEL_CIRCUMFERENCE * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV) ;
    public final double DRVTRAIN_ENC_COUNTS_TO_INCHES_HI = (1/HIGH_GEAR_RATIO) * DRVTRAIN_WHEEL_CIRCUMFERENCE * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV) ; 
   
    public  double         currentEncCountsToInches = 0;
    public  int            currentDrvTrainGear;    
    private DoubleSolenoid gearShiftSolenoid;

    /*----------------------------------------------------------------------------------------------
    *  Analog Pressure Sensor
    *---------------------------------------------------------------------------------------------*/
    private final int    PRESSURE_SENSOR_ANALOG_PORT    = 3; 

    private final double PRESSURE_SENSOR_VOLTAGE_OFFSET = 0.5;

    private final double PRESSURE_SENSOR_VOLTAGE_RANGE  = 4.0;   
    private final double PRESSURE_SENSOR_PRESSURE_RANGE = 200.0;
    private final double PRESSURE_SENSOR_V_TO_PSI = PRESSURE_SENSOR_PRESSURE_RANGE / PRESSURE_SENSOR_VOLTAGE_RANGE;

    private AnalogInput pressureSensor;

    /*----------------------------------------------------------------------------------------------
    *  Autonomous Closed Loop Control - Velocity
    *---------------------------------------------------------------------------------------------*/
    private final int DRVTRAIN_VELOCITY_PID_IDX = 0;
    private final int PID_TIMEOUT_MS            = 10;

    public final double RT_PID_P = 0.05;  
    public final double RT_PID_I = 0.0; 
    public final double RT_PID_D = 0.0;    
    public final double RT_PID_F = 1023.0/20666.0; 

    public final double LT_PID_P = 0.1;  //TBD-MH: verify lt and rt values
    public final double LT_PID_I = 0.0; 
    public final double LT_PID_D = 0.0;    
    public final double LT_PID_F = 1023.0/20666.0;  //TBD-MH:  ADD COMMENTS & DEFINE CONSTANTS

    private      double drvStraightTargetVelocityOffset = -100.0;

    /*----------------------------------------------------------------------------------------------
    *
    *  CatzDriveTrain()
    *
    *---------------------------------------------------------------------------------------------*/
    public CatzDriveTrain() 
    {
        drvTrainMtrCtrlLTFrnt = new WPI_TalonFX(DRVTRAIN_LT_FRNT_MC_CAN_ID);
        drvTrainMtrCtrlLTBack = new WPI_TalonFX(DRVTRAIN_LT_BACK_MC_CAN_ID);

        drvTrainMtrCtrlRTFrnt = new WPI_TalonFX(DRVTRAIN_RT_FRNT_MC_CAN_ID);
        drvTrainMtrCtrlRTBack = new WPI_TalonFX(DRVTRAIN_RT_BACK_MC_CAN_ID);

        //Reset configuration for drivetrain MC's
        drvTrainMtrCtrlLTFrnt.configFactoryDefault();
        drvTrainMtrCtrlLTBack.configFactoryDefault();
        drvTrainMtrCtrlRTFrnt.configFactoryDefault();
        drvTrainMtrCtrlRTBack.configFactoryDefault();

        //Set current limit
        drvTrainCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        drvTrainMtrCtrlLTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlLTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);

        setToCoastMode(); //brake mode set in auton or teleop init

        //Set back Motor's to follow front Motor's
        drvTrainMtrCtrlLTBack.follow(drvTrainMtrCtrlLTFrnt);
        drvTrainMtrCtrlRTBack.follow(drvTrainMtrCtrlRTFrnt);

        //set right motors inverted
        drvTrainMtrCtrlRTFrnt.setInverted(true);
        drvTrainMtrCtrlRTBack.setInverted(true);
        
        //group motors on each side
        drvTrainLT = new MotorControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTBack);
        drvTrainRT = new MotorControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTBack);
        
        gearShiftSolenoid = new DoubleSolenoid(PCM_TYPE, DRVTRAIN_GEARSHIFT_LO_PCM_PORT, DRVTRAIN_GEARSHIFT_HI_PCM_PORT);
        //gearShiftSolenoid = new DoubleSolenoid(PCM_TYPE, DRVTRAIN_GEARSHIFT_HI_PCM_PORT, DRVTRAIN_GEARSHIFT_LO_PCM_PORT); //TBD flipped lo and hi, change later
        shiftToHighGear();

        pressureSensor = new AnalogInput(PRESSURE_SENSOR_ANALOG_PORT);

        setDriveTrainPIDConfiguration(DRVTRAIN_LT, LT_PID_P, LT_PID_I, LT_PID_D, LT_PID_F); //TBD Only used for autonomous
        setDriveTrainPIDConfiguration(DRVTRAIN_RT, RT_PID_P, RT_PID_I, RT_PID_D, RT_PID_F);
        
        //drvTrainMtrCtrlLTFrnt.configOpenloopRamp(0.8);
        //drvTrainMtrCtrlRTFrnt.configOpenloopRamp(0.8);
    
    }   //End of CatzDriveTrain()

    /*----------------------------------------------------------------------------------------------
    *
    *  Closed Loop Control Methods
    *
    *---------------------------------------------------------------------------------------------*/
    public void setDriveTrainPIDConfiguration(int side, double kP, double kI, double kD, double kF) 
    {

        //Configure PID Gain Constants
        if (side == DRVTRAIN_LT) 
        {
             //Configure feedback device for PID loop
            drvTrainMtrCtrlLTFrnt.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, DRVTRAIN_VELOCITY_PID_IDX,
                                                                                                PID_TIMEOUT_MS);
            drvTrainMtrCtrlLTFrnt.config_kP(DRVTRAIN_VELOCITY_PID_IDX, kP);
            drvTrainMtrCtrlLTFrnt.config_kI(DRVTRAIN_VELOCITY_PID_IDX, kI);
            drvTrainMtrCtrlLTFrnt.config_kD(DRVTRAIN_VELOCITY_PID_IDX, kD);
            drvTrainMtrCtrlLTFrnt.config_kF(DRVTRAIN_VELOCITY_PID_IDX, kF);

        } 
        else 
        {
            drvTrainMtrCtrlRTFrnt.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, DRVTRAIN_VELOCITY_PID_IDX, 
                                                                                                PID_TIMEOUT_MS);
            drvTrainMtrCtrlRTFrnt.config_kP(DRVTRAIN_VELOCITY_PID_IDX, kP);
            drvTrainMtrCtrlRTFrnt.config_kI(DRVTRAIN_VELOCITY_PID_IDX, kI);
            drvTrainMtrCtrlRTFrnt.config_kD(DRVTRAIN_VELOCITY_PID_IDX, kD);
            drvTrainMtrCtrlRTFrnt.config_kF(DRVTRAIN_VELOCITY_PID_IDX, kF);
        }
         
    }   //End of setDriveTrainPIDConfiguration()

    /*----------------------------------------------------------------------------------------------
    *
    *  Drive Methods
    *
    *---------------------------------------------------------------------------------------------*/
    public void arcadeDrive(double power, double rotation)
    {
        drvTrainDifferentialDrive.arcadeDrive(-power, rotation);
    }

    

    /*----------------------------------------------------------------------------------------------
    *
    *  Motor Config & Status Methods
    *
    *---------------------------------------------------------------------------------------------*/
    public void setToBrakeMode()
    {
        drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Brake); 
        drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlRTBack.setNeutralMode(NeutralMode.Brake);
    }

    public void setToCoastMode() 
    {
        drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Coast); 
        drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Coast);
        drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Coast);
        drvTrainMtrCtrlRTBack.setNeutralMode(NeutralMode.Coast);
    }

    public double getMotorTemperature(int id)
    {
        double temp = -999.0;
        switch (id)
        {
            case DRVTRAIN_LT_FRNT_MC_CAN_ID:
                temp = drvTrainMtrCtrlLTFrnt.getTemperature();
            break;

            case DRVTRAIN_LT_BACK_MC_CAN_ID:
                temp = drvTrainMtrCtrlLTBack.getTemperature();
            break;

            case DRVTRAIN_RT_FRNT_MC_CAN_ID:
                temp = drvTrainMtrCtrlRTFrnt.getTemperature();
            break;
                
            case DRVTRAIN_RT_BACK_MC_CAN_ID:
                temp = drvTrainMtrCtrlRTBack.getTemperature();
            break;

            default :
                temp = -999.0;          
        }
        return temp; 

    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Gear Shift Methods
    *
    *---------------------------------------------------------------------------------------------*/
    public void shiftToHighGear()
    {
        gearShiftSolenoid.set(Value.kForward);
        currentDrvTrainGear      = DRVTRAIN_GEAR_HI;
        currentEncCountsToInches = DRVTRAIN_ENC_COUNTS_TO_INCHES_HI;
    }

    public void shiftToLowGear()
    {
        gearShiftSolenoid.set(Value.kReverse);
        currentDrvTrainGear      = DRVTRAIN_GEAR_LO;
        currentEncCountsToInches = DRVTRAIN_ENC_COUNTS_TO_INCHES_LO;
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Autonomous - Distance Control
    *
    *---------------------------------------------------------------------------------------------*/
    public void setTargetPosition(double targetPosition)
    {
        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Position, targetPosition);
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Position, targetPosition);
    }

    public double getDrvTrainEncoderCnt(int side) 
    {
        double counts = 0;

        if(side == DRVTRAIN_LT)
        {
            counts = drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(DRVTRAIN_VELOCITY_PID_IDX);   
        }
        else
        {
            counts = drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(DRVTRAIN_VELOCITY_PID_IDX);
        }
        return counts;
    }

    public double getDrvTrainEncoderCntRt() 
    {
        double counts = 0;

        counts = drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(DRVTRAIN_VELOCITY_PID_IDX);   
        
        return counts;
    }

    public double getDrvTrainEncoderCntLt() 
    {
        double counts = 0;

        counts = drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(DRVTRAIN_VELOCITY_PID_IDX);   
        
        return counts;
    }

    public void setIntegratedEncPosition(int position)
    {
        drvTrainMtrCtrlLTFrnt.setSelectedSensorPosition(position);
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Autonomous - Velocity Control
    *
    *---------------------------------------------------------------------------------------------*/
    public void setDrvStraightTargetVelocity(double targetVelocityLT)
    {
        double targetVelocityRT;

        targetVelocityRT = (targetVelocityLT - drvStraightTargetVelocityOffset);
        
        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, targetVelocityLT);
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, targetVelocityRT);
        
    }

    public void setDrvStraightTargetVelocityOffset(double offset)
    {
        drvStraightTargetVelocityOffset = offset;
    }

    public void setTurnInPlaceTargetVelocity(double targetVelocityLT)
    {
        double targetVelocityRT;
        targetVelocityRT = -targetVelocityLT;

        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, targetVelocityLT);
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, targetVelocityRT);
    }

    public double getIntegratedEncVelocity(int side)
    {
        double velocity = -999.0;

        if(side == DRVTRAIN_LT)
        {
            velocity = drvTrainMtrCtrlLTFrnt.getSensorCollection().getIntegratedSensorVelocity(); //returning counts per 100ms
        }
        else if(side == DRVTRAIN_RT)
        {
            velocity = drvTrainMtrCtrlRTFrnt.getSensorCollection().getIntegratedSensorVelocity();
        }
    
        return velocity;
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Analog Pressure Sensor
    *
    *---------------------------------------------------------------------------------------------*/
    public double getAccumulatorPressurePSI()
    {
        double voltage;

        voltage = pressureSensor.getVoltage() - PRESSURE_SENSOR_VOLTAGE_OFFSET;
        return (PRESSURE_SENSOR_V_TO_PSI) * voltage;  
    }   

    public void instantiateDifferentialDrive()
    {
        drvTrainDifferentialDrive = new DifferentialDrive(drvTrainLT, drvTrainRT);
    }

}   //End of public class CatzDriveTrain