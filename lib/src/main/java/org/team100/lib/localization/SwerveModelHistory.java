package org.team100.lib.localization;

import java.util.function.DoubleFunction;

import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * History is just a container, in fact the implementation is little more than a
 * wrapper around TimeInterpolatableBuffer.
 * 
 * The history always has *something* in it, even the initial zero pose.
 * 
 * There are no dependencies managed here; for that, use SwerveModelEstimate.
 * 
 * Note this interface should only be used from within the localization package.
 * 
 * Other SwerveModel consumers should use SwerveModelEstimator.
 */
interface SwerveModelHistory extends DoubleFunction<SwerveModel>{
    @Override
    SwerveModel apply(double timestampSeconds);
}