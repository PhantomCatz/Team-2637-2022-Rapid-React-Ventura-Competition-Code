package frc.DataLogger;

public class CatzLog 
{
    public double robotTime;
    public double robotData1;
    public double robotData2;
    public double robotData3;
    public double robotData4;
    public double robotData5;
    public double robotData6;
    public double robotData7;
    public double robotData8;
    public double robotData9;
    public double robotData10;
    public double robotData11;
    public double robotData12;
    public double robotData13;
    public double robotData14;
    public double robotData15;
  
    
    public int robotDataType;


    public CatzLog(double time, 
                   double data1, double data2, double data3, double data4, double data5, 
                   double data6, double data7, double data8, double data9, double data10, 
                   double data11, double data12, double data13, double data14, double data15)
    {
        robotTime  = time;
        robotData1 = data1;
        robotData2 = data2;
        robotData3 = data3;
        robotData4 = data4;
        robotData5 = data5;
        robotData6 = data6;
        robotData7 = data7;
        robotData8 = data8;
        robotData9 = data9;
        robotData10 = data10;
        robotData11 = data11;
        robotData12 = data12;
        robotData13 = data13;
        robotData14 = data14;
        robotData15 = data15;
    }

  

    public String toString()
    {
        return robotTime +", " + robotData1 + ", " + robotData2 + ", " + robotData3 + ", " + robotData4 + ", " + robotData5 + ", " 
                               + robotData6 + ", " + robotData7 + ", " + robotData8 + ", " + robotData9 + ", " + robotData10 + ","
                               + robotData11 + "," + robotData12 + ", " + robotData13+","  + robotData14+  "," + robotData15;
    }


    

    
}