package frc.robot;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyController{
    //private final int PixyWidth = 316;
    //private final int PixyHeight = 208;
    private final int targetXcenter = 158;
    private final int targetYcenter = 104;
    private final int xErrorMax = 79;
    private final int yErrorMax = 52;
    private final double xP = 1.0/xErrorMax;
    private final double yP = 1.0/yErrorMax;
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
        var biggestBlock = getBiggestBlock();
        if (biggestBlock == null){
            return new double[2];
        }
        biggestBlock.getAngle();
        var xError = biggestBlock.getX() - targetXcenter;
        var yError= biggestBlock.getY() - targetYcenter;
        var rotation = xError * xP;
        var speed = yError * yP;
        rotation = Math.min(rotation, 1);
        rotation = Math.max(rotation, -1);
        speed = Math.min(speed, 1);
        speed = Math.max(speed, -1);
        var speedrotation = new double[2];
        speedrotation[0] = speed;
        speedrotation[1] = rotation;
        return speedrotation;
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