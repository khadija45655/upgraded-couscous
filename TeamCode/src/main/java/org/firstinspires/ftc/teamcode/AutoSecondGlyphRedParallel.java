package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by khadija on 12/1/2017.
 */
@Autonomous(name = "AutoRedParallel2" , group ="Concept")
public class AutoSecondGlyphRedParallel extends Processor {
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

            knockJewel(true);

            //forward(300);
            // go in front of the cyrptograph


            goAngle(30, 0);

            sleep(500);
            turn(177);

            goAngle(12, 0);

            sleep(500);
            gotoColumnLeftEnc();
            stopBotMotors();
            sleep(500);
            score();
            score2();

        }
    }

