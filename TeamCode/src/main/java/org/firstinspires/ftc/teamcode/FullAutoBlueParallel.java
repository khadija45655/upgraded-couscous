package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.lang.Thread.sleep;

/**
 * Created by khadija on 11/25/2017.
 */

@Autonomous(name = "FullAutoBlueParallel" , group ="Concept")

public class    FullAutoBlueParallel extends Processor{
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        checkCol();
        waitForStart();
        checkCol();

        checkVu();


        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(2000);

        runtime.reset();
        while(runtime.milliseconds()<300) {
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        knockJewel(false);

        //forward(300);
        // go in front of the cyrptograph
        //opposite direction of blue

        goAngle(30, 180);

        sleep(500);
        turn(180);

        goAngle(12, 180);

        sleep(500);
        gotoColumnRightEnc();

        stopBotMotors();
        sleep(500);
        score();
    }
}
