package frc.robot;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;

public class PixyController{
    private final Pixy2 _pixy = Pixy2.createInstance(LinkType.SPI);

    public void init(){
        _pixy.init();
    }

    /**
     * Calculates the speed and rotation necessary to track the yellow
     * game ball.
     * @return A double array with the speed in the first slot and rotation
     * in the second slot.
     */
    public double[] trackBall() {

    }
}