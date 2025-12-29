package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Prism.Color;
import org.firstinspires.ftc.teamcode.util.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.Prism.PrismAnimations;


public class Led {

    public static enum State {
        SOLID_RED,
        SOLID_GREEN,
        SOLID_WHITE,
        BLINK_RED,
        BLINK_GREEN
    }

    public GoBildaPrismDriver prism;

    PrismAnimations.Solid red = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid green = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid white = new PrismAnimations.Solid(Color.WHITE);

    PrismAnimations.Blink blinkRed = new PrismAnimations.Blink(Color.RED, Color.WHITE, 200);
    PrismAnimations.Blink blinkGreen = new PrismAnimations.Blink(Color.GREEN, Color.WHITE, 200);


    public Led(HardwareMap hwMap) {
        prism = hwMap.get(GoBildaPrismDriver.class, "led");
        prism.enableDefaultBootArtboard(false);
        prism.clearAllAnimations();
    }

    public void setState(State state) {
        prism.clearAllAnimations();
        switch (state) {
            case SOLID_RED:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, red);
            case SOLID_GREEN:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, green);
            case SOLID_WHITE:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, white);
            case BLINK_RED:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blinkRed);
            case BLINK_GREEN:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blinkGreen);
        }
    }

}
