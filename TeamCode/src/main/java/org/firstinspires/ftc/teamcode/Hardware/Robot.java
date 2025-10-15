package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;

public class Robot extends BaseHardware {

    public DriveTrain driveTrain = new DriveTrain();
    //public Lighting lighting = new Lighting();
    public Sensors sensors = new Sensors();
    public Intake intake = new Intake();
    public Launcher launcher = new Launcher();
    public Spindexer spindexer = new Spindexer();
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        // Must set Hardware Map and telemetry before calling init
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();


         //  lighting.hardwareMap = this.hardwareMap;
        //lighting.telemetry = this.telemetry;
       // lighting.init();

        sensors.hardwareMap = this.hardwareMap;
        sensors.telemetry = this.telemetry;
        sensors.init();

        intake.hardwareMap = this.hardwareMap;
        intake.telemetry = this.telemetry;
        intake.init();

        launcher.hardwareMap = this.hardwareMap;
        launcher.telemetry = this.telemetry;
        launcher.init();


        spindexer.hardwareMap = this.hardwareMap;
        spindexer.telemetry = this.telemetry;
        spindexer.init();

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        //lighting.init_loop();
        sensors.init_loop();
        intake.init_loop();
        launcher.init_loop();
        spindexer.init_loop();
    }

    @Override
    public void start() {
        driveTrain.start();
       // lighting.start();
        sensors.start();
        intake.start();
        launcher.start();
        spindexer.start();


       // lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    @Override
    public void loop() {
        driveTrain.loop();
       //. lighting.loop();
        sensors.loop();
        intake.loop();
        launcher.loop();
        spindexer.loop();



    }


    @Override
    public void stop() {
        driveTrain.stop();
       // lighting.stop();
        sensors.stop();
        intake.stop();
        launcher.stop();
        spindexer.stop();

       // lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }




}
