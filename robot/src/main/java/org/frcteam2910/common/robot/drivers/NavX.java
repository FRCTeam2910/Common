package org.frcteam2910.common.robot.drivers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.frcteam2910.common.drivers.Gyroscope;

public final class NavX extends Gyroscope {
    private final AHRS navX;

    public NavX(SPI.Port port) {
        this(port, (byte) 200);
    }

    public NavX(SPI.Port port, byte updateRate) {
        navX = new AHRS(port, updateRate);
    }

    @Override
    public void calibrate() {
        navX.reset();
    }

    @Override
    public double getUnadjustedAngle() {
        return navX.getAngle();
    }

    @Override
    public double getUnadjustedRate() {
        return navX.getRate();
    }
}
