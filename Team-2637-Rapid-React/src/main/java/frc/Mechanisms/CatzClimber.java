package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.RelativeEncoder;

import frc.DataLogger.*;
import frc.robot.*;


import edu.wpi.first.wpilibj.Timer;

public class CatzClimber
{
    public static WPI_TalonFX elevatorMtr;

    public static LimitSwitchNormal elevatorTopLimitSwitch;
    public static LimitSwitchNormal elevatorBtmLimitSwitch;

    private final int ELEVATOR_MC_CAN_ID     = 50;
    public final int ELEVATOR_MC_PDP_PORT     = 2;

    private final static double ELEVATOR_MTR_EXTEND_POWER  = 1.0;
    private final static double ELEVATOR_MTR_RETRACT_POWER = -0.5;
    private final static double ELEVATOR_MTR_STOP          = 0.0;

    public final static double FIRST_CLIMB_STAGE_DISTANCE  = 0.0;
    public final static double SECOND_CLIMB_STAGE_DISTANCE = 0.0;
    public final static double THIRD_CLIMB_STAGE_DISTANCE  = 0.0;
    public final static double FOURTH_CLIMB_STAGE_DISTANCE = 0.0;
    
    private static double elevatorEncoderCnt;
    private final double ELEVATOR_GEAR_RATIO = 25.0/1.0;
    private final static double TALONFX_INTEGRATED_ENC_CNTS_PER_REV = 2048.0;
    private final double ELEVATOR_COUNTS_TO_INCHES = (1/ELEVATOR_GEAR_RATIO) * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV);

    public static Timer elevatorMtrTimer;

    private double climbElevatorMtrPwr;

    //rotating arm 
    public static CANSparkMax rotatingArmMtr;

    private final int ROTATING_ARM_MC_CAN_ID = 51;
    public final int ROTATING_ARM_MC_PDP_PORT = 4;

    public static SparkMaxLimitSwitch rotatingArmLimitSwitch; //make it plugged into the robo rio

    //arm extending motors ; ratio = 30:1 or 1:30
    public static CANSparkMax leftExtendingArmMtr;
    public static CANSparkMax rightExtendingArmMtr;

    private final int LT_EXT_ARM_MC_CAN_ID  = 53;
    private final int RT_EXT_ARM_MC_CAN_ID  = 52;
    public final int LT_EXT_ARM_MC_PDP_PORT = 10;
    public final int RT_EXT_ARM_MC_PDP_PORT = 6;

    //rotation values
    public final double ROTATING_ARM_HOOK_CLEAR_POSITION   = 70.0;
    public final double ROTATING_ARM_HOOK_ALIGNED_POSITION = ROTATING_ARM_HOOK_CLEAR_POSITION - 10.0;
    public final double HOME_POSITION                      = 0.0;

    public static RelativeEncoder rotatingArmEncoder;

    private final static double CANSPARKMAX_ENC_COUNTS_TO_REV = 42.0;
    private final double ROTATING_ARM_GEAR_RATIO              = 35.0/1.0;//5.0/1.0; // double check if gear ratio is still 5.0/1.0
    private final double ROTATING_ARM_COUNTS_TO_DEGREES       = (1/ROTATING_ARM_GEAR_RATIO) * (1/CANSPARKMAX_ENC_COUNTS_TO_REV);

    public final static double ROTATING_ARM_DOWN_PWR    = 0.5;
    public final static double ROTATING_ARM_RETRACT_PWR = -0.5;
    public final static double ROTATING_ARM_OFF_PWR     = 0.0;

    public static Timer rotatingArmTimer;

    private Thread elevatorThread;

    public CatzLog data;
    boolean uniqueFirstLogData = true;


    public CatzClimber()
    {
        elevatorMtr = new WPI_TalonFX(ELEVATOR_MC_CAN_ID);
        elevatorMtr.configFactoryDefault();
        elevatorMtr.setNeutralMode(NeutralMode.Brake);

        //Forward = elevator down, Reverse = elevator up
        elevatorMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        //elevatorMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        elevatorMtr.configForwardSoftLimitEnable(false);
        //elevatorMtr.configReverseSoftLimitEnable(true);

        elevatorMtr.set(TalonFXControlMode.Position, 0.0);        

        rotatingArmMtr = new CANSparkMax(ROTATING_ARM_MC_CAN_ID, MotorType.kBrushless);
        rotatingArmMtr.restoreFactoryDefaults();
        rotatingArmMtr.setIdleMode(IdleMode.kBrake);

        rotatingArmLimitSwitch = rotatingArmMtr.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        rotatingArmLimitSwitch.enableLimitSwitch(true);

        rotatingArmEncoder = rotatingArmMtr.getEncoder();
        rotatingArmEncoder.setPosition(HOME_POSITION);
        rotatingArmEncoder.setPosition(360.0/ROTATING_ARM_COUNTS_TO_DEGREES);

        leftExtendingArmMtr = new CANSparkMax(LT_EXT_ARM_MC_CAN_ID, MotorType.kBrushless);
        leftExtendingArmMtr.restoreFactoryDefaults();
        leftExtendingArmMtr.setIdleMode(IdleMode.kBrake);

        rightExtendingArmMtr = new CANSparkMax(RT_EXT_ARM_MC_CAN_ID, MotorType.kBrushless);
        rightExtendingArmMtr.restoreFactoryDefaults();
        rightExtendingArmMtr.setIdleMode(IdleMode.kBrake);

        //rightExtendingArmMtr.follow(leftExtendingArmMtr, true);

        elevatorMtrTimer = new Timer();

        //setElevatorPosition();
    }

    public void setElevatorPosition()
    {
        elevatorThread = new Thread(() ->
        {
            double currentTime;

            elevatorMtrTimer.reset();
            elevatorMtrTimer.start();

            /*data = new CatzLog(elevatorMtrTimer.get(), 2, 3, 4, 5, 6,
                                    -999.0, -999.0, -999.0, -999.0, -999.0,
                                    -999.0, -999.0, -999.0, -999.0, -999.0);
            Robot.dataCollection.logData.add(data);*/ ///TBD Need to figure out why this doesn't make data collection work

            while(true)
            {
                currentTime = elevatorMtrTimer.get();
                
                //double countsToInchesConversionFactor = 12/TALONFX_INTEGRATED_ENC_CNTS_PER_REV;
                //double currentPosition = elevatorMtr.getSelectedSensorPosition(0) * countsToInchesConversionFactor;

                if (Robot.xboxAux.getAButton())
                {
                    data = new CatzLog(currentTime, climbElevatorMtrPwr, elevatorMtr.getMotorOutputPercent(),
                                                                         elevatorMtr.getMotorOutputVoltage(), 
                                                                         elevatorMtr.getSupplyCurrent(),
                                                                         elevatorMtr.getSelectedSensorPosition(0),
                                                                         rightExtendingArmMtr.getOutputCurrent(), rightExtendingArmMtr.getBusVoltage(),
                                                                         leftExtendingArmMtr.getOutputCurrent(), leftExtendingArmMtr.getBusVoltage(),
                                                                         rotatingArmMtr.getOutputCurrent(), rotatingArmMtr.getBusVoltage(),
                                                                         leftExtendingArmMtr.getMotorTemperature(), rightExtendingArmMtr.getMotorTemperature(), -999.0, -999.0);
                    Robot.dataCollection.logData.add(data);        
                    
                }

                Timer.delay(0.02);
            }
        });
        elevatorThread.start();
    }


    public static boolean extendElevatorToTop(double timeoutSeconds)
    {
        double currentTime = 0.0;
        boolean elevatorExtendDone = false;
        boolean done = false;

        elevatorMtrTimer.reset();
        elevatorMtrTimer.start();

        elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_EXTEND_POWER);

        while(done == false)
        {
            currentTime = elevatorMtrTimer.get();

            if(currentTime > timeoutSeconds)
            {
                elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                done = true;
            }
            
        }
        return elevatorExtendDone; //Should be done or set extendDone = done
    }


    public void moveElevatorToPosition(double distanceInInches, double timeoutSeconds)
    {
        double currentTime            = 0.0;
        boolean retractElevatorDone = false;

        elevatorMtrTimer.reset();
        elevatorMtrTimer.start();
        
        double countsToInchesConversionFactor = 12/TALONFX_INTEGRATED_ENC_CNTS_PER_REV; //placeholder 
        while (retractElevatorDone == false)
        {
            
            double currentPosition = elevatorMtr.getSelectedSensorPosition(0) * countsToInchesConversionFactor;
            
            if (distanceInInches < 0.0)
            {
                elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_RETRACT_POWER);

                if (currentPosition >= distanceInInches)
                {
                    elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                    retractElevatorDone = true;
                }              
            } 
            else if(distanceInInches > 0.0) 
            {
                elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_EXTEND_POWER);

                if (currentPosition >= distanceInInches)
                {
                    elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                    retractElevatorDone = true;
                }
            }

            if(currentTime > timeoutSeconds)
            {
                elevatorMtr.set(ControlMode.PercentOutput, ELEVATOR_MTR_STOP);
                retractElevatorDone = true; 

            }
        }
    }

    public static void moveRotatingArmToPosition(double targetAngle, double timeoutSeconds)
    {
        double currentTime    = 0.0;
        boolean rotatingArmLoopDone = false;

        double countsToDegreesConversionFactor = 360/CANSPARKMAX_ENC_COUNTS_TO_REV;
        double initialRotatingArmAngle = rotatingArmEncoder.getPosition();

        rotatingArmTimer.reset();
        rotatingArmTimer.start(); 
        
        while(rotatingArmLoopDone == false)
        {
            double currentAngle = rotatingArmEncoder.getPosition() * countsToDegreesConversionFactor;

            if(targetAngle < initialRotatingArmAngle)
            {
                rotatingArmMtr.set(ROTATING_ARM_RETRACT_PWR);

                if(currentAngle == targetAngle)
                {
                    rotatingArmMtr.set(ROTATING_ARM_OFF_PWR);
                    rotatingArmLoopDone = true;
                }
            }

            if(targetAngle > initialRotatingArmAngle)
            {
                rotatingArmMtr.set(ROTATING_ARM_DOWN_PWR);

                if(currentAngle == targetAngle)
                {
                    rotatingArmMtr.set(ROTATING_ARM_OFF_PWR);
                    rotatingArmLoopDone = true;
                }
            }
            
            if(rotatingArmLimitSwitch.isPressed())
            {
                rotatingArmMtr.set(ROTATING_ARM_OFF_PWR);
                rotatingArmLoopDone = true;
            }

            if(currentTime > timeoutSeconds)
            {
                rotatingArmMtr.set(ROTATING_ARM_OFF_PWR);
                rotatingArmLoopDone = true;
            }
        }
    }

    public void climbElevatorManualCntrl(double motorPower)
    {
        climbElevatorMtrPwr = motorPower;

        //System.out.println(climbElevatorMtrPwr);

        elevatorMtr.set(climbElevatorMtrPwr);
        
        
    }

    public void climbRotatingArmManualCntrl(double motorPower)
    {
        rotatingArmMtr.set(motorPower);
    }

    public void elevatorMtrPwrOff()
    {
        elevatorMtr.set(ELEVATOR_MTR_STOP);
    }

    public void climbExtendRotatingArmManualCtrl(double motorPower)
    {
        leftExtendingArmMtr.set(motorPower);

        // Right arm is faster than left, so modify right arm speeds
        // Right arm speed is different going down vs up
        if(motorPower > 0)
        {
            rightExtendingArmMtr.set(-motorPower * 0.85);
        }
        else
        {
            rightExtendingArmMtr.set(-motorPower * 0.875);
        }
    }

    public void climbExtendRightRotatingArmManualCtrl(double motorPower)
    {
        rightExtendingArmMtr.set(-motorPower);
    }

    public void climbExtendLeftRotatingArmManualCtrl(double motorPower)
    {
        leftExtendingArmMtr.set(motorPower);
    }

    public void setToCoastMode()
    {
        elevatorMtr.setNeutralMode(NeutralMode.Coast);
    }

    public void firstClimbStage()
  {
    moveElevatorToPosition(FIRST_CLIMB_STAGE_DISTANCE, 2);
    //retractLightSaber(true, 2);
    moveRotatingArmToPosition(45, 0.5);
  }

  public void secondClimbStage()
  {
    //ExtendLightSaber(true, 2);
    moveElevatorToPosition(SECOND_CLIMB_STAGE_DISTANCE, 2);
    moveRotatingArmToPosition(0, 1);
  }

  public void thirdClimbStage()
  {
    //retractLightSaber(true, 2);
    moveElevatorToPosition(THIRD_CLIMB_STAGE_DISTANCE, 2);
    moveRotatingArmToPosition(45, 2);
  }

  public void lastClimbStage()
  {
    //ExtendLightSaber(true, 2);
    moveElevatorToPosition(FOURTH_CLIMB_STAGE_DISTANCE, 2);
    moveRotatingArmToPosition(0, 2);
  }
}