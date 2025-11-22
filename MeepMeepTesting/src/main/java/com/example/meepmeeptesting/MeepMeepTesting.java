package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark()) // set our bot to be red
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -15, Math.PI))
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(215)) // position to shoot zero batch

                .strafeToLinearHeading(new Vector2d(35,-30), Math.toRadians(270)) // go to first set of artifacts
                .strafeToLinearHeading(new Vector2d(35,-50), Math.toRadians(270)) // drive into first set of artifacts
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)) // go back after grabbing first set of artifacts to shoot

                .strafeToLinearHeading(new Vector2d(12,-30), Math.toRadians(270)) // go to second set of artifacts
                .strafeToLinearHeading(new Vector2d(12,-50), Math.toRadians(270)) // drive into second set of artifacts

                .turnTo(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,-58), Math.toRadians(180)) // push classifier gate

                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)) // go back after grabbing second set of artifacts to shoot

                .strafeToLinearHeading(new Vector2d(-12,-30), Math.toRadians(270)) // go to third set of artifacts
                .strafeToLinearHeading(new Vector2d(-12,-50), Math.toRadians(270)) // drive into third set of artifacts
                .strafeToLinearHeading(new Vector2d(55,-15), Math.toRadians(195)) // go back after grabbing third set of artifacts to shoot

                .build());


        RoadRunnerBotEntity alliancePartnerBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark()) // set the other bot to be blue
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        alliancePartnerBot.runAction(alliancePartnerBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(52)))
                .strafeToLinearHeading(new Vector2d(-45,-45), Math.toRadians(52))
                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(alliancePartnerBot)
                .start();
    }
}