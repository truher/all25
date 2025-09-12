# Calibration

I tried capturing some checkerboards images and using mrcal, using the stock lens mount on the "DEV" camera.

The data is in the team drive, under Vision/calibration 2025/CAL.

The resulting calibration is as follows, using the OPENCV8 lensmodel.

'intrinsics': [ 942.6615503, 939.0811468, 704.561148, 525.6324672, -0.2763529592, 0.1063535989, 0.0003014940077, 0.0003963850429, 0.01972759901, 0.03351011343, -0.03883095374, 0.07544202732,],

this array matches the expectations of OpenCV: fx,fy,cx,cy,distortion0,distortion1,....


I also tried OPENCV4:

'intrinsics': [ 941.7704704, 938.3253293, 705.0534812, 526.3332502, -0.291696437, 0.09250522071, 0.0002758218827, 0.0004190938935,],

This is similar, but not identical, to the 8-parameter model.

Our calibration from last season, which I did using the one-dimentional rig, was this:

[935, 935, 728, 544, -0.374, 0.1, 0, 0]
