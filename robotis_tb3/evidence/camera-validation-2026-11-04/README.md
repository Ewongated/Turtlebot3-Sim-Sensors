# Project 11 camera evidence

See [protocol, results and limits](../../CAMERA_VALIDATION.md).

- summary.json: fitted/configured intrinsics, held-out errors, marker results and source hashes.
- checkerboard-fixtures.json: all 24 board poses and detection outcomes.
- checker-view-*.png: all calibration images; views 0–17 train and 18–23 held out.
- calibration-scene.xml and checkerboard.png: independent render fixture.
- marker-observations.csv: all 360 observations, including misses and negative controls.
- camera-validation.png: residuals versus range and image noise.

These are offline simulation fixtures. They are not physical calibration or
additional navigation-success trials. The assessed detector/parameters were unchanged.
