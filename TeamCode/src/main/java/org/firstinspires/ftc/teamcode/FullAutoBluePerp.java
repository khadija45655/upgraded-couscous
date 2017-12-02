package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by khadija on 11/25/2017.
 */

@Autonomous(name = "FullAutoBluePerp" , group ="Concept")

public class FullAutoBluePerp extends Processor{

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        checkCol();
        waitForStart();
        checkCol();

        checkVu();


        bot.glyphServo1.setPosition(0.69);
        bot.glyphServo2.setPosition(0.35);
        sleep(1000);


        runtime.reset();
        while(runtime.milliseconds()<300) {
            bot.slideMotor.setPower(-.8);
        }
        bot.slideMotor.setPower(0);

        knockJewel(false);

        //forward(300);
        // get off the stone\\
        goAngle(40,180);

        turn(-90);

        gotoColumnRightEnc();// need to come at the column from the right for this to work

        stopBotMotors();
        sleep(1000);
        score();

    }
}
