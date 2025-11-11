package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;

import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * See INTERCEPT.md for details.
 */
public class Intercept {
    /**
     * Find a turret rotation which will intercept the target. If more than one
     * solution is possible, choose the sooner one. If no solution is possible,
     * return Optional.empty().
     * 
     * @param robotPosition  field-relative robot position, meters
     * @param robotVelocity  field-relative robot velocity, meters/sec
     * @param targetPosition field-relative target position, meters
     * @param targetVelocity field-relative target velocity, meters/sec
     * @param muzzleSpeed    speed of the projectile, meters/sec
     * @return field-relative firing solution azimuth
     */
    public static Optional<Rotation2d> intercept(
            Translation2d robotPosition,
            GlobalVelocityR2 robotVelocity,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity,
            double muzzleSpeed) {
            
                double T0x = targetPosition.getX() - robotPosition.getX();
                double T0y = targetPosition.getY() - robotPosition.getY();

                double vTx = targetVelocity.x() - robotVelocity.x();
                double vTy = targetVelocity.y() - robotVelocity.y();
                double T0_dot_vT = T0x * vTx + T0y * vTy;
                double vT_dot_vT = Math.pow(vTx, 2) + Math.pow(vTy, 2);

                double C = Math.pow(T0x, 2) + Math.pow(T0y, 2);

                
                double B = 2.0 * T0_dot_vT;
                double A = vT_dot_vT - Math.pow(muzzleSpeed, 2);
                List<Double> solutions = Math100.solveQuadratic(A, B, C);
                double theta = 0;
                if(solutions.size()>0){
                    for( int i=0; i<solutions.size(); i++){
                        double t = solutions.get(i);
                        double Ix = T0x + vTx * t;
                        double Iy = T0y + vTy * t;
                        theta = Math.atan2(Iy,Ix);
                    }
                   
                }

        

        return Optional.of(new Rotation2d(theta));
    }

}
