package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyController{
    //private final int PixyWidth = 316;
    //private final int PixyHeight = 208;
    private final int targetXcenter = 158;
    private final Pixy2 _pixy = Pixy2.createInstance(LinkType.SPI);
    private final PIDController _rotationPid = new PIDController(.001,0,0);

    public void init(){
        _pixy.init();
        _rotationPid.setSetpoint(0);
        _rotationPid.setTolerance(2);
    }

    /**
     * Calculates the speed and rotation necessary to track the yellow
     * game ball.
     * @return A turn value between -1 and 1.
     */
    public double trackBall() {
        var biggestBlock = getBiggestBlock();
        if (biggestBlock == null){
            return 0;
        }
        biggestBlock.getAngle();
        var xError = biggestBlock.getX() - targetXcenter;
        var rotation = _rotationPid.calculate(xError);
        rotation = Math.min(rotation, 1);
        rotation = Math.max(rotation, -1);
        return rotation;
    }

    public void turnLightOn(int r, int g, int b){
        _pixy.setLamp((byte)1, (byte)1);
        _pixy.setLED(r, g, b);
    }

    public void turnLightOff(){
        _pixy.setLamp((byte)0, (byte)0);
    }

    private Block getBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 15, for a slight increase in efficiency
		int blockCount = _pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 15);
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = _pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if ((block.getWidth()*block.getHeight()) > (largestBlock.getWidth()*largestBlock.getHeight())) {
				largestBlock = block;
			}
		}
		return largestBlock;
	}
}