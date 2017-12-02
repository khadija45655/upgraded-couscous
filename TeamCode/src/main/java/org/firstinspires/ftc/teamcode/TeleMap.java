package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by khadija on 11/25/2017.
 */

public class TeleMap {
    public BNO055IMU imu;
    HardwareMap hwMap = null;
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor slideMotor;
    DcMotor relicMotor;


    Servo glyphServo1;
    Servo glyphServo2;
    Servo jewelServo;
    Servo relicFingers;
    Servo relicWrist;



    ModernRoboticsI2cRangeSensor rangeSensor = null;

    ColorSensor colorSensor = null;


    RelicRecoveryVuMark columnToScore;


    ElapsedTime runtime = new ElapsedTime();


    public void init(HardwareMap ahwMap) {

        // save reference to HW Map
        hwMap = ahwMap;
        imu = hwMap.get(BNO055IMU.class, "imu");
        motorLB = hwMap.dcMotor.get("motorLB");
        motorLF = hwMap.dcMotor.get("motorLF");
        motorRF = hwMap.dcMotor.get("motorRF");
        motorRB = hwMap.dcMotor.get("motorRB");
        relicMotor = hwMap.dcMotor.get("relicMotor");

        slideMotor = hwMap.dcMotor.get("slideMotor");
        //motorLB.setDirection(DcMotor.Direction.REVERSE);
        //^^^^^^^^^^^^^^^^^^^^^^^^^R^RmotorRF.setDirection(DcMotor.Direction.REVERSE);
        glyphServo1 = hwMap.servo.get("glyphServo1");
        glyphServo2 = hwMap.servo.get("glyphServo2");
        //jewelServo = hwMap.servo.get("jewelServo");
        relicFingers = hwMap.servo.get("relicFingers");
        relicWrist = hwMap.servo.get("relicWrist");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class,"rangeSensor");
        colorSensor = hwMap.get(ColorSensor.class,"colorSensor");



        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorLB.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// could be a drain on power

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //jewelServo.setPosition(.5);
        //glyphServo1.setPosition(0.4);
        //glyphServo2.setPosition(0.6);


    }


}
