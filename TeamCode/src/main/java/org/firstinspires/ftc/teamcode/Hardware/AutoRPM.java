package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//Auto calculates RPM for launcher
public class AutoRPM {

    public Telemetry telemetry = null;

    public HardwareMap hardwareMap = null;




    //private Limey limey;
    //private Launcher launcher;
    public Limey limey;
    public Launcher launcher;

    public boolean shouldMeasure = false;
    public double MeasureRPMTOP = 0;

    public void init(){

    }

    public void init_loop(){

    }

    public void start(){

    }

    public void loop(){

    }

    public void stop(){

    }


    public AutoRPM(Limey limey, Launcher launcher){
        this.limey = limey;
        this.launcher = launcher;

    }

    public void update(){
        double tx = limey.getTx();
        double ty = limey.getTy();
        double yaw = limey.getTagAngle();


        double[] rpms = calculateRPMs(tx, ty, yaw);

        launcher.setTargetRPMs(rpms[0],rpms[1]);
    }
    public double[] calculateRPMs(double tx, double ty, double yaw){

        double distance = limey.getTagDistance();

       /* double baseTop = 5970;
        double baseBottom = 5980;

        double distanceAdjust = ty * -25;
        double angleAdjust = Math.abs(tx) * 10;
        double yawAdjust = Math.abs(yaw) * 5;

        */
        // Top motor
        double d1 = 0.5;    //in meters
        double r1top = 2000;    //need to update test

        double d2 = 2.9;    //in meters
        double r2top = 3600;

        double m_top = (r2top - r1top) / (d2 - d1);
        double b_top = r1top - m_top * d1;

        double targetTopRPM = m_top * distance + b_top;

        // bottom motor
       // double d1b = 18;
        double r1bottom = 4100;

       // double d2b = 180;
        double r2bottom = 5750;

        double m_bottom = (r2bottom - r1bottom) / (d2 - d1);
        double b_bottom = r1bottom - m_bottom * d1;

        double targetBottomRPM = m_bottom * distance + b_bottom;

        return new double[]{targetTopRPM, targetBottomRPM};

    }







}
