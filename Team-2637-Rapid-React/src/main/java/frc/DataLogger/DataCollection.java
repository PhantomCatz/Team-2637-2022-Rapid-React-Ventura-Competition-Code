package frc.DataLogger;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;
//import frc.Autonomous.CatzAutonomous;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.String;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

public class DataCollection 
{	
    Date date = Calendar.getInstance().getTime();	
    SimpleDateFormat sdf = new SimpleDateFormat("_yyyyMMdd_kkmmss");	
    String dateFormatted = sdf.format(date);

    public boolean fileNotAppended = false;

    //decide the location and extender
    public final String logDataFilePath = "//media//sda1//RobotData";
    public final String logDataFileType = ".csv";

    private       Thread  dataThread;

    public static boolean logDataValues = false;   
    public static int     logDataID;

    public ArrayList<CatzLog> logData;

    StringBuilder sb = new StringBuilder();

    private final double LOG_SAMPLE_RATE = 0.1;

    public final int LOG_ID_DRV_TRAIN          = 1;
    public final int LOG_ID_DRV_STRAIGHT_PID   = 2;
    public final int LOG_ID_DRV_DISTANCE_PID   = 3;
    public final int LOG_ID_DRV_TURN_PID       = 4;
    public final int LOG_ID_SHOOTER            = 5;
    public final int LOG_ID_DRV_STRAIGHT       = 6;
    public final int LOG_ID_DRV_TURN           = 7;
    public final int LOG_ID_CLIMB              = 8;

    public boolean validLogID = true;

    private final String LOG_HDR_DRV_TRAIN = "time,pdp-v,dt-lf-I,dt-lb-I,dt-rf-I,dt-rb-I,dt-lf-T,dt-lb-T,dt-rf-T,dt-rb-T,dt-l-ie,dt-r-ie,dt-l-ee,dt-r-ee,dt-l-v,dt-r-v";
    private final String LOG_HDR_DRV_STRAIGHT_PID  = "pid-p,pid-i,pid-d,pid-f,pid-iz,cnt-to-in,in-hi-gear,trgt-dst" +
                                                           "tarVel, lt-acc-Vel, rt-acc-vel, lt-vel-er, rt-vel-er,lt-enc-ct,rt-enc-ct,dist";
    private final String LOG_HDR_DRV_DISTANCE_PID = "Undefined";
    private final String LOG_HDR_DRV_TURN_PID = "Undefined";
    private final String LOG_HDR_SHOOTER = "time, traceID, RPM-top, Vel-top, vel-err-top, mtr-pwr-top, vout-top, curr-top,RPM-bot, Vel-bot, vel-err-bot, mtr-pwr-bot, vout-bot, curr-bot, steady-counter";
    private final String LOG_HDR_DRV_STRAIGHT = "time, Velocity, rt-curr-Vel, rt-vel-er, rt-enc-cnt,lt-curr-Vel,lt-vel-er,lt-enc-cnt, dist,cur-ang";
    private final String LOG_HDR_DRV_TURN = "cur-time, del-time, cur-ang, cur-err, del-error, curr-pow";
    private final String LOG_HDR_CLIMB    = "cur-time, tar-pos, cur-pos, ra-enc";
    public String logStr;

    public void setLogDataID(final int dataID)
    {
        logDataID = dataID;
    
    }

    
    public void dataCollectionInit(final ArrayList<CatzLog> list)
    {   
        date = Calendar.getInstance().getTime();
        sdf = new SimpleDateFormat("_yyyyMMdd_kkmmss");	
        dateFormatted = sdf.format(date);

        logData = list;

        dataThread = new Thread( () ->
        {
            while(!Thread.interrupted())
            {   
                if(logDataValues == true)
                {
                    collectData(logDataID);
                } 
                else if (logDataValues == false) 
                {
                 
                } 

                Timer.delay(LOG_SAMPLE_RATE);

            }

        } );

        dataThread.start();
    }

    public void startDataCollection() 
    {
        logDataValues = true;
    }

    public void stopDataCollection() 
    {
        logDataValues = false; 
    }

    public void  collectData(final int dataID)
    {
        CatzLog data;
        double data1 = -999.0;
        double data2 = -999.0;
        double data3 = -999.0;
        double data4 = -999.0;
        double data5 = -999.0;
        double data6 = -999.0;
        double data7 = -999.0;
        double data8 = -999.0;
        double data9 = -999.0;
        double data10 = -999.0;
        double data11 = -999.0;
        double data12 = -999.0;
        double data13 = -999.0;
        double data14 = -999.0;
        double data15 = -999.0;
        //double data16 = -999.0;

        
        //define each data
        switch (dataID) 
        {
            case LOG_ID_DRV_TRAIN :
                data1 = Robot.pdp.getVoltage();

                data2 = Robot.pdp.getCurrent(Robot.driveTrain.DRVTRAIN_LT_FRNT_MC_PDP_PORT);
                data3 = Robot.pdp.getCurrent(Robot.driveTrain.DRVTRAIN_LT_BACK_MC_PDP_PORT);
                data4 = Robot.pdp.getCurrent(Robot.driveTrain.DRVTRAIN_RT_FRNT_MC_PDP_PORT);
                data5 = Robot.pdp.getCurrent(Robot.driveTrain.DRVTRAIN_RT_BACK_MC_PDP_PORT);

                data6 = Robot.driveTrain.getMotorTemperature(Robot.driveTrain.DRVTRAIN_LT_FRNT_MC_CAN_ID);
                data7 = Robot.driveTrain.getMotorTemperature(Robot.driveTrain.DRVTRAIN_LT_BACK_MC_CAN_ID);
                data8 = Robot.driveTrain.getMotorTemperature(Robot.driveTrain.DRVTRAIN_RT_FRNT_MC_CAN_ID); 
                data9 = Robot.driveTrain.getMotorTemperature(Robot.driveTrain.DRVTRAIN_RT_BACK_MC_CAN_ID); 

                //data10 = Robot.driveTrain.getIntegratedEncPosition("LT");
                //data11 = Robot.driveTrain.getIntegratedEncPosition("RT");

                //data12 = Robot.driveTrain.getSrxMagPosition("LT");
                //data13 = Robot.driveTrain.getSrxMagPosition("RT");

                data14 = Robot.driveTrain.getIntegratedEncVelocity(Robot.driveTrain.DRVTRAIN_LT);
                data15 = Robot.driveTrain.getIntegratedEncVelocity(Robot.driveTrain.DRVTRAIN_RT);

                break;
     
            case LOG_ID_SHOOTER:
            case LOG_ID_DRV_STRAIGHT:
            case LOG_ID_DRV_TURN:
            case LOG_ID_CLIMB: 
                break;
           


            default :
                validLogID = false;

        }
        //record data
        if(validLogID == true) 
        {
            data = new CatzLog(Robot.dataCollectionTimer.get(), data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12, data13, data14, data15);
            logData.add(data);
        }
    }
    
    public void writeHeader(PrintWriter pw) 
    {
        switch (logDataID)
        {
            case LOG_ID_DRV_TRAIN:
                pw.printf(LOG_HDR_DRV_TRAIN);
                break;
            case LOG_ID_DRV_STRAIGHT_PID:
                pw.printf(LOG_HDR_DRV_STRAIGHT_PID);
                break;
            case LOG_ID_DRV_DISTANCE_PID:
                pw.printf(LOG_HDR_DRV_DISTANCE_PID);
                break;
            case LOG_ID_DRV_TURN_PID:
                pw.printf(LOG_HDR_DRV_TURN_PID);
                break;
            case LOG_ID_SHOOTER:
                pw.printf(LOG_HDR_SHOOTER);
                break;    
            case LOG_ID_DRV_STRAIGHT:
                pw.printf(LOG_HDR_DRV_STRAIGHT);
                break;
            case LOG_ID_DRV_TURN:
                pw.printf(LOG_HDR_DRV_TURN);
            case LOG_ID_CLIMB:
                pw.printf(LOG_HDR_CLIMB);
            default :
                pw.printf("Invalid Log Data ID");            


        }
    }
    
    //create log file
    public String createFilePath()
    {
	String logDataFullFilePath = logDataFilePath + dateFormatted + logDataFileType;
    	return logDataFullFilePath;
    }

    // print out data after fully updated
    public void exportData(ArrayList<CatzLog> data) throws IOException
    {   
        System.out.println("Export Data ///////////////");    
        try (
            
        FileWriter     fw = new FileWriter(createFilePath(), fileNotAppended);
        BufferedWriter bw = new BufferedWriter(fw);
        PrintWriter    pw = new PrintWriter(bw))

        {
            writeHeader(pw);
            pw.print("\n");

            // loop through arraylist and adds it to the StringBuilder
            int dataSize = data.size();
            for (int i = 0; i < dataSize; i++)
            {
                pw.print(data.get(i).toString() + "\n");
                pw.flush();
            }

            pw.close();
        }
    }
}