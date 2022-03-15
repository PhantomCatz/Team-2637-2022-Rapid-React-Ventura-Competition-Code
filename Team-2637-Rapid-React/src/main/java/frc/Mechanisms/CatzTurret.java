package frc.Mechanisms;

import frc.robot.Robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.*;

public class CatzTurret
{
    private final int TURRET_MC_CAN_ID = 20;

    public CANSparkMax turretMC;

    public boolean inAutonomous = false;

    public final int TURRET_STATE_MANUAL            = 0;
    public final int TURRET_STATE_IDLE              = 1;
    public final int TURRET_STATE_MOVE_TO_POSITION  = 2;
    public final int TURRET_STATE_MOVE_TO_SHOOT_POS = 3;

    public int turretState = TURRET_STATE_IDLE;

    public static final double ZERO_VELOCITY = 0.0;

    public final double MAX_POSITION  =  100.0;
    public final double MIN_POSITION  = -100.0;
    public final double HOME_POSITION = 0.0;
    public final double SECURITY_STOP_RESET_ROTATION = 0.5;

    public final int SHOOTING_ANGLE = 10;

    public final int TURRET_GEARBOX_VERSA_1          = 5;
    public final int TURRET_GEARBOX_VERSA_2          = 4;
    public final int TURRET_GEARBOX_TURRET_GEAR      = 140 / 10;
    public final int TURRET_GEAR_REDUCTION           = TURRET_GEARBOX_VERSA_1 * TURRET_GEARBOX_VERSA_2 * TURRET_GEARBOX_TURRET_GEAR; //Equals 280

    private final int TURRET_MC_CURRENT_LIMIT      = 40;

    public RelativeEncoder turretEncoder;
    public double turretPositionDeg;
    
    public final double MIN_TURRET_PWR =  0.065;

    public final double TURRET_LOW_RATE  = 0.15;
    public final double TURRET_HIGH_RATE = 0.6; // if want to set max pwr: TURRET_HIGH_RATE = 1

    public double targetPosOffset      =  1.0; 
    public double targetPos            = 45.0; 

    public double turretPwrReduce;
    public double turretPosAngleError;

    public final boolean DONT_STOP_ON_LS_HIT = false;
    public final boolean STOP_ON_LS_HIT      =  true;

    public DigitalInput limitSwitchHome;
    public SparkMaxLimitSwitch magLimitSwitchLeft;
    public SparkMaxLimitSwitch magLimitSwitchRight;

    public final int LIMIT_SWITCH_HOME_PORT = 0;

    //public SparkMaxLimitSwitch limitSwitchAutonShoot;

    public final double TURRET_THREAD_WAITING_TIME = 0.010;
    private Thread turretThread;

    boolean uniqueFirstLogData = true;
   
    double currentTime = 0.0;
    public Timer turrettimer;

    public final double MOVE_TO_POSITION_CYCLE_TIMEOUT_SEC = 1.0; 
    public double moveToPositionTimeoutCnt = (MOVE_TO_POSITION_CYCLE_TIMEOUT_SEC / TURRET_THREAD_WAITING_TIME) + 1;
    public int moveToPositionTimeoutCntr = 0;        //TBD - SHOULD RENAME TO moveToPositionTimeoutCntr SO YOU CAN TELL THEY ARE RELATED
    
    public double thresholdPositive;
    public double thresholdNegative; 
    public double currentPos;
    public double turretMtrPwr;

    public double pidTurnDecelAngle = 10;
    public double pidTurnDecelRate = 0.5;

    public boolean targetPosReached = false;

    public CatzLog data;

    public CatzTurret()
    {
        turretMC = new CANSparkMax(TURRET_MC_CAN_ID, MotorType.kBrushless);

        //reset configuration
        turretMC.restoreFactoryDefaults();

        //set MC to idle brake mode
        turretMC.setIdleMode(IdleMode.kCoast);

        limitSwitchHome       = new DigitalInput(LIMIT_SWITCH_HOME_PORT); //moved due to wiring issue on 2/6
        magLimitSwitchLeft    = turretMC.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        magLimitSwitchRight   = turretMC.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        //limitSwitchAutonShoot = turretMC.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen); 

        magLimitSwitchLeft.enableLimitSwitch(STOP_ON_LS_HIT);
        magLimitSwitchRight.enableLimitSwitch(STOP_ON_LS_HIT);
        //limitSwitchAutonShoot.enableLimitSwitch(DONT_STOP_ON_LS_HIT);

        turretEncoder = turretMC.getEncoder();
        turretEncoder.setPositionConversionFactor(360.0/TURRET_GEAR_REDUCTION);

        turretMC.setSmartCurrentLimit(TURRET_MC_CURRENT_LIMIT);

        turretEncoder.setPosition(HOME_POSITION);

        turrettimer = new Timer();

        turrettimer.start();

        turretControl();
        
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  turretControl()
    *
    *---------------------------------------------------------------------------------------------*/
    public void turretControl()
    {  

        turretEncoder.setPosition(HOME_POSITION);
        
        turretThread = new Thread(() ->
        {
        
            while(true)
            {
                currentTime = turrettimer.get();
                switch(turretState)
                {
                    case TURRET_STATE_MANUAL:   //TBD - COMBINE WITH IDLE STATE?
                        //Do nothing when joystick controlled
                        //System.out.println("Manual");
                    break;

                    case TURRET_STATE_IDLE:
                        //no input -> stationary
                        //System.out.println("OUT");
                    break;
                    
                    case TURRET_STATE_MOVE_TO_POSITION:
                        currentPos = getTurretPositionDeg();

                        turretPosAngleError = Math.abs(targetPos - currentPos);

                        //System.out.println("A:");
                        
                        if (currentPos >= thresholdNegative && currentPos <= thresholdPositive)
                            {
                                stopTurret();
                                targetPosReached = true;        //TBD - DO WE NEED THIS?
                                //System.out.println("B: " + currentPos);
                            }
                        else 
                        {
                            //System.out.println("C"); 
                            if (moveToPositionTimeoutCntr > moveToPositionTimeoutCnt)
                            {
                                stopTurret();
                                //System.out.println("D");
                            }
                            else
                            {
                               // System.out.println("E");
                                if (turretPosAngleError < pidTurnDecelAngle) 
                                {
                                   // System.out.println("F");
                                    if(Math.abs(turretMtrPwr) > MIN_TURRET_PWR)
                                    {
                                        turretMtrPwr = turretMtrPwr * pidTurnDecelRate;

                                        //set motor power to minimum power if it is under minimun while decelerating
                                        if (Math.abs(turretMtrPwr) < MIN_TURRET_PWR)
                                        {
                                            turretMtrPwr = MIN_TURRET_PWR;
                                        }
                                        turretMC.set(turretMtrPwr);
                                    }   
                                }
                            }
                            moveToPositionTimeoutCntr++;
                            //System.out.println("angle: " + currentPos + " pwr: " + turretMtrPwr);  

                            /*if (uniqueFirstLogData == true)
                            {
                                data = new CatzLog(targetPos, targetPosOffset,  turretMtrPwr, pidTurnDecelRate, pidTurnDecelAngle, 
                                                                                                                moveToPositionTimeoutCnt, 
                                                                                                                turretEncoder.getPositionConversionFactor(),
                                                    -999.0, -999.0, -999.0, -999.0, -999.0, 
                                                    -999.0, -999.0, -999.0, -999.0 );   //set to 16 values
                                uniqueFirstLogData = false;
                            }
                            else
                            {
                                data = new CatzLog(currentTime, currentPos, turretPosAngleError, turretMtrPwr, positionCycleCounter,
                                                                                                               turretMC.getOutputCurrent(),
                                                                                                               turretMC.getBusVoltage(), 
                                                                                                               turretEncoder.getVelocity(),
                                                                                                               -999.0, -999.0, -999.0, -999.0, 
                                                                                                               -999.0, -999.0, -999.0, -999.0);   //set to 16 values
                            }
                            Robot.dataCollection.logData.add(data);    */         
                        }
                    break;

                    default:
                    break;
                }

                //checkIfAtHardstop();
                
                if (limitSwitchHome.get() == true)
                {
                    turretEncoder.setPosition(HOME_POSITION);
                }


                Timer.delay(TURRET_THREAD_WAITING_TIME);
            }
        });

        turretThread.start();
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  setTargetPos()
    *
    *---------------------------------------------------------------------------------------------*/
    public void setTargetPos(double target)
    {
        targetPos = target;
        thresholdPositive = targetPos + targetPosOffset;
        thresholdNegative = targetPos - targetPosOffset;

        currentPos = getTurretPositionDeg();
        turretPosAngleError = Math.abs(targetPos - currentPos);

        //edit deceleration numbers to make rotation smoother and less jerky
        if(turretPosAngleError <= 10.0)
        {
            turretMtrPwr = MIN_TURRET_PWR;
            pidTurnDecelRate   = 0.55; 
            pidTurnDecelAngle  = 1.0; 
            moveToPositionTimeoutCnt = ( 0.7 / TURRET_THREAD_WAITING_TIME) + 1;

        }
        else if(turretPosAngleError <= 25.0)
        {
            turretMtrPwr = .3;
            pidTurnDecelRate   = 0.7; 
            pidTurnDecelAngle  = 3.0; 
            moveToPositionTimeoutCnt = ( 0.7 / TURRET_THREAD_WAITING_TIME) + 1;

        }
        else if(turretPosAngleError <= 50)
        {
            turretMtrPwr = .5;
            pidTurnDecelRate   = 0.55; 
            pidTurnDecelAngle  = 8.0; 
            moveToPositionTimeoutCnt = (1.0 / TURRET_THREAD_WAITING_TIME) + 1;
        }
        else if(turretPosAngleError <= 100.0)
        {
            turretMtrPwr       = .7;
            pidTurnDecelRate   = 0.75;
            pidTurnDecelAngle  = 10.0;
            moveToPositionTimeoutCnt = (1.5 / TURRET_THREAD_WAITING_TIME) + 1;

        }
        else if(turretPosAngleError <= 150.0)
        {
            turretMtrPwr       = .8;
            pidTurnDecelRate   = 0.93;
            pidTurnDecelAngle  = 5.0;
        }
        else // (turretPosAngleError > 150.0)
        {
            turretMtrPwr       = .8;
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 5.0;
        }

        if(targetPos < currentPos)
        {
            turretMtrPwr = -turretMtrPwr;
        }
        
        moveToPositionTimeoutCntr = 0;

        turretState = TURRET_STATE_MOVE_TO_POSITION;
        turretMC.set(turretMtrPwr);
    }




    


    /*----------------------------------------------------------------------------------------------
    *
    *  rotateTurret()
    *   - Manual control in Teleop
    *
    *---------------------------------------------------------------------------------------------*/
    public void rotateTurret(double mtrPwr)
    {
        turretMtrPwr = mtrPwr;
        turretMC.set(turretMtrPwr);
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  stopTurret()
    *
    *---------------------------------------------------------------------------------------------*/
    public void stopTurret()
    {
        turretMC.set(ZERO_VELOCITY);

        turretState = TURRET_STATE_IDLE;
    }



    /*----------------------------------------------------------------------------------------------
    *
    *      moveToHome()
    *
    *---------------------------------------------------------------------------------------------*/
    public void moveToHome()
    {
        setTargetPos(HOME_POSITION);
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  getTurretPositionDeg()
    *
    *---------------------------------------------------------------------------------------------*/
    public double getTurretPositionDeg()
    {
        turretPositionDeg = turretEncoder.getPosition();    //*(360/TURRET_GEAR_REDUCTION);
        return turretPositionDeg;
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  turretStartup()  TBD - USED ANYWHERE???
    *
    *---------------------------------------------------------------------------------------------*/
    public void turretStartup(double target)
    {
        turretEncoder.setPosition(HOME_POSITION);
    
        targetPos = target;

        turretState = TURRET_STATE_MOVE_TO_POSITION;
    }


    public void smartDashboardTurret_DEBUG()
    {
        double turretPos = turretEncoder.getPosition();
        SmartDashboard.putNumber("Turret Motor Revolutions", turretPos);
        SmartDashboard.putNumber("Turret Motor Position", turretPos * 360);
        SmartDashboard.putNumber("Turret Rotation", getTurretPositionDeg());
        
        //SmartDashboard.putBoolean("Auton Shooter LS Pressed", limitSwitchAutonShoot.isPressed());
        SmartDashboard.putNumber("Turret State", moveToPositionTimeoutCntr);

        SmartDashboard.putNumber("TurretVel", turretEncoder.getVelocity());

        //SmartDashboard.putBoolean("leftMag", magLimitSwitchLeft.isPressed());
        //SmartDashboard.putBoolean("rightMag", magLimitSwitchRight.isPressed());
    }

    public void smartDashboardTurret()
    {
        SmartDashboard.putBoolean("Home LS", limitSwitchHome.get());
        SmartDashboard.putNumber("Turret Angle", getTurretPositionDeg());
    }

}