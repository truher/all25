"""Factory for AprilTag detectors.

One type of factor uses constant camera parameters, the other treats
camera parameters as a model variable, for calibration.

This also serves as a template for parameterized estimators -- in this
case, the parameter is the landmark location.

Note that the camera model here doesn't clip, it will happily return pixel
values that are outside the camera frame (i.e. negative).
"""

# pylint: disable=C0103,E0611,E1101,R0913
from typing import Callable

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel  # type:ignore

from app.pose_estimator.numerical_derivative import (
    numericalDerivative31,
    numericalDerivative32,
    numericalDerivative33,
)



def h_fn(
    landmark: np.ndarray,
) -> Callable[[gtsam.Pose2, gtsam.Pose3, gtsam.Cal3DS2], np.ndarray]:
    """Returns a pixel estimation function for a constant landmark,
    with variable pose, offset, and calibration.
    The landmark is the field position of a tag corner."""

    def h(p0: gtsam.Pose2, offset: gtsam.Pose3, calib: gtsam.Cal3DS2) -> np.ndarray:
        """Estimated pixel location of the target."""
        camera_pose = gtsam.Pose3(p0).compose(offset) 
        camera = gtsam.PinholeCameraCal3DS2(camera_pose, calib)
        return camera.project(landmark)

    return h


def h_H(
    landmark: np.ndarray,
    measured: np.ndarray,
    p0: gtsam.Pose2,
    offset: gtsam.Pose3,
    calib: gtsam.Cal3DS2,
    H: list[np.ndarray],
) -> np.ndarray:
    """Error function (in pixels) including Jacobian vector, H."""
    h = h_fn(landmark)
    result = h(p0, offset, calib) - measured
    if H is not None:
        H[0] = numericalDerivative31(h, p0, offset, calib)
        H[1] = numericalDerivative32(h, p0, offset, calib)
        H[2] = numericalDerivative33(h, p0, offset, calib)
    return result


def factorCustom(
    landmark: np.ndarray,
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
    offset_key: int,
    calib_key: int,
) -> gtsam.NoiseModelFactor:
    """using variable offset and calibration, use this for camera calibration.
    landmark: field coordinates
    measured: pixel coordinate
    model: pixel noise"""

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        """Pull the variables from the model and apply the px error function."""
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        offset: gtsam.Pose3 = v.atPose3(this.keys()[1])
        calib: gtsam.Cal3DS2 = v.atCal3DS2(this.keys()[2])

        return h_H(landmark, measured, p0, offset, calib, H)

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([p0_key, offset_key, calib_key]),  # type:ignore
        error_func,
    )


def factorNative(
    landmark: np.ndarray,
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
    offset_key: int,
    calib_key: int,
) -> gtsam.NoiseModelFactor:
    return gtsam.PlanarProjectionFactor3(
        p0_key, offset_key, calib_key, landmark, measured, model
    )


def factor(
    landmark: np.ndarray,
    measured: np.ndarray,
    model: SharedNoiseModel,
    p0_key: int,
    offset_key: int,
    calib_key: int,
) -> gtsam.NoiseModelFactor:
    return factorNative(landmark, measured, model, p0_key, offset_key, calib_key)
    # return factorCustom(landmark, measured, model, p0_key, offset_key, calib_key)