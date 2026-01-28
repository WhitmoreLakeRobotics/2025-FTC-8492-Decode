package org.firstinspires.ftc.teamcode.Hardware;

public class LimeyLaunch {

    //private Limey limey;
    //private Launcher launcher;
    public Limey limey = new Limey();
    public Launcher launcher = new Launcher();

    public boolean shouldMeasure = false;


    public LimeyLaunch(Limey limey,Launcher launcher){
        this.limey = limey;
        this.launcher = launcher;

    }

    public void cmdAutoMeasure(){

    }

    public void update(){
        double tx = limey.getTx();
        double ty = limey.getTy();
        double yaw = limey.getTagAngle();


        double[] rpms = calculateRPMs(tx, ty, yaw);

        //launcher.setTargetRPMs(rpms[0],rpms[1]);
    }
    private double[] calculateRPMs(double tx, double ty, double yaw){

        double distance = limey.getTagDistance();

       /* double baseTop = 5970;
        double baseBottom = 5980;

        double distanceAdjust = ty * -25;
        double angleAdjust = Math.abs(tx) * 10;
        double yawAdjust = Math.abs(yaw) * 5;

        */
        // Top motor
        double d1 = 18;    //in inches
        double r1top = 5999;    //need to update test

        double d2 = 168;    //in inches currently 14 feet
        double r2top = 4078;

        double m_top = (r2top - r1top) / (d2 - d1);
        double b_top = r1top - m_top * d1;

        double targetTopRPM = m_top * distance + b_top;

        // bottom motor
       // double d1b = 18;
        double r1bottom = 666;

       // double d2b = 180;
        double r2bottom = 3227;

        double m_bottom = (r2bottom - r1bottom) / (d2 - d1);
        double b_bottom = r1bottom - m_bottom * d1;

        double targetBottomRPM = m_bottom * distance + b_bottom;

        return new double[]{targetTopRPM, targetBottomRPM};

    }




}
