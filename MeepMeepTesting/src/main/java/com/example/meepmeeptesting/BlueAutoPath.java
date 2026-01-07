package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueAutoPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark()) // set our bot to be red
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(53,-12), Math.toRadians(270)) // position to shoot zero batch
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(35,-30), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(35,-50), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(270)) // go back after grabbing first set of artifacts to shoot
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(12,-30), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(12,-50), Math.toRadians(270)) // drive into second set of artifacts

//

                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(270)) // go back after grabbing second set of artifacts to shoot
                        .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(38,-62), Math.toRadians(-10)) // wall set
                .strafeToLinearHeading(new Vector2d(60,-62), Math.toRadians(-10)) // drive in

                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(270)) // go back after grabbing wall set
                        .waitSeconds(3)


                .build());






        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}