package frc.Mechanisms;

import java.nio.ByteBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiDAR
{
    private I2C lidar;

    public double range = -1.0;

    //registers in hex from LiDAR Documentation : https://static.garmin.com/pumac/LIDAR-Lite%20LED%20v4%20Instructions_EN-US.pdf
    private final int LIDAR_DEFAULT_ADDRESS    = 0x62;
    private final int REGISTER_FACTORY_RESET   = 0xE4;
    private final int REGISTER_ACQ_COMMANDS    = 0x00;
    private final int REGISTER_STATUS          = 0x01;
    private final int REGISTER_FULL_DELAY_HIGH = 0x11;
    private final int REGISTER_FULL_DELAY_LOW  = 0x10;

    //delay between status reads during distance measurement
    private final double LIDAR_STATUS_LOOP_DELAY_SEC = 0.002;

    private final double CONV_CM_TO_IN = 0.3937;

    //variables for getLiDARRange()
    private byte       mask;
    private ByteBuffer regValue;
    private int        distInBits;
    private int        value;
    private boolean    done;

    private boolean runDebugOnce;

    public boolean inRange = false;

    public boolean manualOveride = false;

    Thread liDARThread;

    public LiDAR()
    {
        //initialization for LiDAR
        lidar = new I2C(I2C.Port.kMXP, LIDAR_DEFAULT_ADDRESS);
        
        //factory resets LiDAR
        lidar.write(REGISTER_FACTORY_RESET, 0x01);

        //----------------setup variables for getLiDARRange()-------------------------\\
        //mask set to 0000 0001 to compare status register value to
        mask = Byte.decode("0x01");

        //buffer allocated 1 byte
        regValue = ByteBuffer.allocate(0x01);


        runDebugOnce = false;

        startLiDAR();
    }

    public void startLiDAR() 
    {

        liDARThread = new Thread(() -> //start of thread
        {
            while(true)
            {
                //constantly updates LiDAR measurement, in its own thread due to getting loop time overrun warnings.
                getLidarRange();
                //getLiDARDebug();
            }
            
        }); 
        //end of thread
        liDARThread.start();
    }



    public void getLidarRange()
    {

        done = false;

        //Writes to the LiDAR to start a measurement
        lidar.write(REGISTER_ACQ_COMMANDS, 0x04);

        /*
            Loop to wait for status register to return LOW instead of HIGH, which means that the measurement has been
            completed. This is done by reading the register status, which will give 0x01 (0000 0001) if HIGH and 0x00
            (0000 0000) if low. Then it is compared to the mask, which has a value of 0x01 (0000 0001) with the & 
            command, which compares each byte and only returns 1 if both values are 1. So if the register reads 
            HIGH, then the value will be set to 0x01 (0000 0001), and if the register reads LOW, value will be set to
            0x00 (0000 0000) which means that it has finished taking the measurement and will end the loop.
        */
        while(done == false)
        {
            regValue.clear();

            lidar.read(REGISTER_STATUS, 0x01, regValue);
      
            value = mask & regValue.get();

            if (value == 0x00)
            {
            done = true;
            }

        Timer.delay(LIDAR_STATUS_LOOP_DELAY_SEC);
        }

        /*
            tbd?
        */
        regValue.clear();
        lidar.read(REGISTER_FULL_DELAY_HIGH, 0x01, regValue);
        distInBits = regValue.get() << 8;

        regValue.clear();
        lidar.read(REGISTER_FULL_DELAY_LOW, 0x01, regValue);
        distInBits = distInBits | regValue.get();

        range = ((double)distInBits * CONV_CM_TO_IN);    
    }

    public boolean isInRange(double maxRange, double minRange)
    {
        if(manualOveride == true)
        {
            manualOveride = false;
            return true;
            
        }
        else
        {
            if(range < maxRange && range > minRange)
            {
                inRange = true;
                return true;
            }
            else
            {
                inRange = false;
                return false;
            }
        } 
    }

  //-------------------SmartDashboard-------------------\\

    public void getLiDARDebug()
    {   
        //to only get the port and device address once
        if(runDebugOnce)
        {
            SmartDashboard.putNumber("LiDAR Port", lidar.getPort());
            SmartDashboard.putNumber("LiDAR Device Address", lidar.getDeviceAddress());

            runDebugOnce = false;
        }

        SmartDashboard.putNumber("LiDAR Range", range);
        SmartDashboard.putBoolean("cargoInRange", inRange);

    }

    public void smartDashboardLiDAR()
    {
        SmartDashboard.putNumber("LiDAR Range", range);

    }

}