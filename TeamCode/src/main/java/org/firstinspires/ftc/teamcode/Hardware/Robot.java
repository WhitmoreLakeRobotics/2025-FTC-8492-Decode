package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Common.CommonLogic;
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
    public HuskyLens huskyLens = new HuskyLens();
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

    //auto align constants
    public double minTargetVertPos = 0;
    public double minTargetDist = 10;
    public double maxTargetVertPos = 200;
    public double maxTargetDist = 72;
    public double minTargetVertPos = 65; //63-69
    public double minTargetDist = 26;
    public double maxTargetVertPos = 169;
    public double maxTargetDist = 78;

    public double nominalTagWidthRatio = 1;
    public double nominalTagWidthRatio = 0.95;
    public double nominalTagAngle = 0;
    public double extremeTagWidthRatio = 0.5;
    public double extremeTagAngle = 45;
    public double tagExtremeRightPos = 300;
    public double tagExtremeRightAngle = 45;
    public double targetPointFromTag = 10;
    public double extremeTagWidthRatio = 0.6829;
    public double extremeTagAngle = 75;
    public double tagExtremeRightPos = 296;
    public double tagExtremeRightAngle = 65;
    public double targetPointFromTag = 12;


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

        huskyLens.hardwareMap = this.hardwareMap;
        huskyLens.telemetry = this.telemetry;
        huskyLens.init();

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
        huskyLens.init_loop();
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
        huskyLens.start();


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
        huskyLens.loop();



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
        huskyLens.stop();

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
/*
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
        */
public double targetDistanceCalc(){
  /* double tagVertResult = Tag.y;
   double tagVertRatio = (tagVertResult - minTargetVertPos)/(maxTargetVertPos - minTargetVertPos);

   double tagDistCalc = minTargetDist + ((maxTargetDist - minTargetDist)*tagVertRatio);

   Return tagDistCalc;
*/
}
public double targetAngleCalc(){
    double currentTargetPos = Tag.x;
    double targetAngle = tagExtremeRightAngle * ((currentTargetPos - 160)/tagExtremeRightPos - 160);

    double currentTargetRatio = Tag.height/Tag.width;
    double tagAngle = extremeTagAngle * ((currentTargetRatio - nominalTagWidthRatio)/(extremeTagWidthRatio - nominalTagWidthRatio));
    double targetDistanceCalc = targetDistanceCalc();
    double hypotenuse = Math.sqrt((targetDistanceCalc * targetDistanceCalc) + targetPointFromTag * targetPointFromTag - 2*(targetDistanceCalc*targetPointFromTag*Math.cos(tagAngle))

    double compensationAngle = 180 - tagAngle - (Math.asin(Math.sin(tagAngle)*targetDistance)/hypotenuse));
    double defaultAngle = 25;

    if (driveTrain.getCurrentHeading() >= 90) {
        return defaultAngle;
    } else if (driveTrain.getCurrentHeading() <= -90) {
        return -defaultAngle;
    } else if (huskyLens.){
        //compensate left
        return driveTrain.getCurrentHeading() + targetAngle - compensationAngle;
    } else if (bluetag id){
        //compensate right
        return driveTrain.getCurrentHeading() + targetAngle + compensationAngle;
    }  else {

    }

}


}











