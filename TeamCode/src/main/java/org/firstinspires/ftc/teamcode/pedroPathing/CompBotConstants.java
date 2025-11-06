package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//this is for adjusting the competition robot.
// you need to change which constants file the code points to for the different robots.
public class CompBotConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
           .forwardZeroPowerAcceleration(-32.48245754) //changed from (-35.4224625953)
            .lateralZeroPowerAcceleration(-78.22727075) ///chabged from (-43.517320985)
// org .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.008, 0.01))
           .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.0))
//org            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.01, 0.025))
          .headingPIDFCoefficients(new PIDFCoefficients(1, 0.003, 0.06, 0.025))
           // .useSecondaryTranslationalPIDF(true)
           // .useSecondaryHeadingPIDF(true)
           // .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,.0001,0,0.01))
            .centripetalScaling(.0006)
            .mass(12.7);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //this should be at max for tuning
            .rightFrontMotorName("RDM1")
            .rightRearMotorName("RDM2")
            .leftRearMotorName("LDM2")
            .leftFrontMotorName("LDM1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
           .xVelocity(76.06906841)
           .yVelocity(57.86164852);

 //   public Pose myOffset = new Pose(6.0, -0.1875, Math.toRadians(180));
    public static OTOSConstants localizerConstants =  new OTOSConstants()
            .hardwareMapName("otto")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-5.5, -2.25, 0))
//            .offset(myOffset) 2.25 5.5
           .linearScalar(1.035446186) //Multiplier
           .angularScalar(0.997716667) ;//Multiplier




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1.25, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }

}
