# Detection

How does the AprilTag detector work?

Read through [apriltag.c](https://github.com/AprilRobotics/apriltag/blob/master/apriltag.c) to see.  It's a little bit hard to follow, due to the multithreading.

* The entry point is `apriltag.c/apriltag_detector_detect()`. 
  * "Decimate" the input.  this means to sample every Nth pixel, to reduce the size of the image, to make quad detection faster, in exchange for worse detection at long range, which is not a good tradeoff for us, so this is 1.0 (no decimation)
  * Blur or sharpen.  the parameter is the std dev of the blur kernel.  a little blur seems to help with small tags.  using a negative number here means to blur by the absolute value of the parameter and subtract the blurred version from the original.
  * Write the `debug_preprocess.pnm`, which shows the blurred image
  * Find possible tag outlines (`apriltag_quad_thresh.c/apriltag_quad_thresh()`)
    * Threshold the image.  this means to turn every pixel into either black or white, depending on whether they're below the local (four-pixel ) average brightness, or above.  low-contrast areas are ignored and colored gray.
    * Write `debug_threshold.pnm`, which shows the thresholded image.
    * Find "connected components" which means to clump adjacent black and white pixels togther into shapes.
    * Write `debug_segmentation.pnm`, which colors the image to show the outline of each "clump".
    * Create "gradient clusters," which are groups of pixels at the boundaries between white and black pixels, i.e. they describe the clump outlines.  each point also includes the direction of the gradient, towards the white pixel, which is used later.
    * Write the `debug_clusters.pnm`, which shows the outlines.
    * Try to find quadrilateral-shaped clump outline clusters (`fit_quad()`).
      * Find the center of the cluster, and compute a sort key ("slope", a misnomer) that increases clockwise around the center.
      * Make sure the gradient points in the correct direction (for "normal" tags with a black boundary surrounded by white, this means the gradient is pointing away from the center).
      * Sort the points so that they wrap around the center sequentially.
      * Try to fit line segments to small groups of (~30) points.
      * Use the worst-fitting line segments as candidate tag corners.
        * For each candidate corner, try to fit the points between it and the next corner to a straight line segment.
        * Reject the segment if the fit isn't good enough.
        * Reject the segment if the angle at the corner is too small or too large.
        * If we have four good segments, we have a candidate tag.
      * Fit the lines between the corners again.
      * Find the corners (intersections of the fits).
    * Make sure the tag size is at least 8 pixels (for 36h11).
    * Check the corner angles again.
    * Write `debug_lines.ps`, which shows the quad geometry as a (very simple) postscript file.  The coordinate system in the PS file is image pixels, so you can just read it.
  * Write `debug_quads_raw.pnm`, which should show the same thing as the ps file above, overlaid onto the image
  * Attempt to decode each quad as a tag (`quad_decode_task()`)
    * Refit each edge and the corners, if we "decimated" to find the corners initially; this fit uses all the pixels.
    * Solve the homography of unit-measure tag corners to the corners in pixels.
    * Try to decode it (`quad_decode()`)
      * Use the homography to project the sample location of each cell in the tag
      * Use the tag edge to calibrate "black" and "white" levels
      * Read the value of each cell sequentially as a binary code
      * Look up the code (`quick_decode_codeword()`)
    * Project the tag center.
    * Rearrange the corners so they're in the right order.
  * Write `debug_samples.pnm` which shows the projected sample location for each bit in the code key.
  * Write `debug_quads_fixed.pnm`, which shows the quads after the above operations; the main difference would be the refitting.
  * If overlapping detection ids are the same, keep the best one.
  * Write `debug_output.ps`, which shows the detection corners.  The difference is that this shows the corners in the canonical order.
  * Write `debug_output.pnm`, which shows the same corners overlaid on the image.
  * Write `debug_quads.ps`, which shows all the quads, even if they're not detections.
  * Sort the detections by id.



--------------------

Another description of the algorithm, which appears to be out of date:

https://medium.com/@hirok4/apriltag-detection-algorithm-f1f562434c9