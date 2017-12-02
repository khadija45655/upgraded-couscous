package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by khadija on 11/29/2017.
 */

@Autonomous(name = "Auto ODS" , group ="Concept")

public class Ods1 extends Processor{

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        bot.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
            bot.jewelServo.setPosition(.4);
            telemetry.addData("range", bot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("range mm ",bot.rangeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("colorsß blue", bot.colorSensor.blue());
            telemetry.addData("colorsß red", bot.colorSensor.red());

            telemetry.update();

        }
    }
}
