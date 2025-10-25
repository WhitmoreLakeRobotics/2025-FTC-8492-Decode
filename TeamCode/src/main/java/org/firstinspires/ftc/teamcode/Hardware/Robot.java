package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.Supplier;

public class Robot extends BaseHardware {

    private static final Logger log = LoggerFactory.getLogger(Robot.class);
    public DriveTrain driveTrain = new DriveTrain();
    //public Lighting lighting = new Lighting();
   // public Sensors sensors = new Sensors();
    public Intake intake = new Intake();
    public Launcher launcher = new Launcher();
   // public Spindexer spindexer = new Spindexer();
    //public Flickiteer flickiteer = new Flickiteer();
    public TransitionRoller transitionRoller = new TransitionRoller();
    public LauncherBlocker launcherBlocker = new LauncherBlocker();
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public boolean bCkSenors = false;

    @Override
    public void init() {
        // Must set Hardware Map and telemetry before calling init
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();


         //  lighting.hardwareMap = this.hardwareMap;
        //lighting.telemetry = this.telemetry;
       // lighting.init();

       // sensors.hardwareMap = this.hardwareMap;
        // sensors.telemetry = this.telemetry;
       // sensors.init();

        intake.hardwareMap = this.hardwareMap;
        intake.telemetry = this.telemetry;
        intake.init();

        launcher.hardwareMap = this.hardwareMap;
        launcher.telemetry = this.telemetry;
        launcher.init();

         /*
        spindexer.hardwareMap = this.hardwareMap;
        spindexer.telemetry = this.telemetry;
        spindexer.init();

          */

        //flickiteer.hardwareMap = this.hardwareMap;
        //flickiteer.telemetry = this.telemetry;
        //flickiteer.init();

        launcherBlocker.hardwareMap = this.hardwareMap;
        launcherBlocker.telemetry = this.telemetry;
        launcherBlocker.init();

        transitionRoller.hardwareMap = this.hardwareMap;
        transitionRoller.telemetry = this.telemetry;
        transitionRoller.init();

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        //lighting.init_loop();
       // sensors.init_loop();
        intake.init_loop();
        launcher.init_loop();
      //  spindexer.init_loop();
       // flickiteer.init_loop();
        launcherBlocker.init_loop();
        transitionRoller.init_loop();
    }

    @Override
    public void start() {
        driveTrain.start();
       // lighting.start();
       // sensors.start();
        intake.start();
        launcher.start();
      //  spindexer.start();
        //flickiteer.start();
        launcherBlocker.start();
        transitionRoller.start();


       // lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    @Override
    public void loop() {
        driveTrain.loop();
       //. lighting.loop();
       // sensors.loop();
        intake.loop();
        launcher.loop();
       // spindexer.loop();
        //flickiteer.loop();
        launcherBlocker.loop();
        transitionRoller.loop();



    }


    @Override
    public void stop() {
        driveTrain.stop();
       // lighting.stop();
       // sensors.stop();
        intake.stop();
        launcher.stop();
      //  spindexer.stop();
       // flickiteer.stop();
        launcherBlocker.stop();
        transitionRoller.stop();

       // lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

      public void safteyCheck(){
        //when called comfirm flicker is in safe position before spindexing.
      }
/*
    public void Cksem (){
        if(bCkSenors){
            sensors. SpindexerSlot1 = sensors.getSlotArtifact(sensors.SDC01);
            sensors.SpindexerSlot2 = sensors.getSlotArtifact(sensors.SDC02);
            sensors.SpindexerSlot3 = sensors.getSlotArtifact(sensors.SDC03);
            sensors.IntakeSlot = sensors.getSlotArtifact(sensors.NTKC01);
        }
    }  */

public void LaunchNear(){         //wait for launcher to spin up to speed.
        launcher.cmdOutnear();
     if (launcher.bAtSpeed) {
         launcherBlocker.cmdUnBlock();
         if(launcherBlocker.AtUnBlocked == true){
             transitionRoller.cmdSpin();
         }
     }
}

public void LaunchFar(){          //wait for launcher to spin up to speed.
        launcher.cmdOutfar();
      if (launcher.bAtSpeed){
        launcherBlocker.cmdUnBlock();
          if (launcherBlocker.AtUnBlocked == true){
            transitionRoller.cmdSpin();
        }
      }
}

public void NoLaunch(){
    transitionRoller.cmdStop();
    launcherBlocker.cmdBlock();
        launcher.cmdStop();
}

}
