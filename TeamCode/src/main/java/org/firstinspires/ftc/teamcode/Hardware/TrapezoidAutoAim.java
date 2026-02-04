package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autons.TestAuton;
import org.firstinspires.ftc.teamcode.Tele_Op;

import java.util.MissingFormatWidthException;
import java.util.Objects;

@Disabled
public class TrapezoidAutoAim {

    private Limey limey;
    private Turret turret;

    public Mode CurrentMode;

    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public void init(){

    }

    public void init_loop(){

    }

    public void start(){

    }

    public void loop(){
        //runtime.log("Position");
        limey.getTx();
        if(limey.getTx() >= 72){
            turret.cmdLeft();
        } else if (limey.getTx() <= 72){
            turret.cmdRight();
        }else{
            turret.cmdNo();
        }

        if(CurrentMode == Mode.Targeting && limey.getTagID() == -1){
            CurrentMode = Mode.Target_NotFound;
        }





    }

    public void stop(){

    }



    public enum Mode{
        Targeting,
        Target_Acquired,
        Target_NotFound,
        NotTrying

    }

}



