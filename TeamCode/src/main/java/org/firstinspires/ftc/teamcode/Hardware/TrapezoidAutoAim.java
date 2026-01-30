package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class TrapezoidAutoAim {

    Robot robot = new Robot();

    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    public void init(){

    }

    public void init_loop(){

    }

    public void start(){

    }

    public void loop(){
        runtime.log("Position");


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



