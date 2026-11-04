# Project 11 video

[Play/download project-11-demo.mp4](project-11-demo.mp4)

The five-minute H.264 MP4 is included directly in this public repository. It is a
silent explanatory video, with on-screen text instead of a voice track. The supplied
brief does not specify voice narration or faces. [Optional narration script](Narration-script.txt).

The nominal and camera-noise/dropout clips are complete actual simulation recordings
at 1x speed. Two render views were captured from the assessed pipeline without
changing physics, random draws, observations or controller commands. The recorded
metrics reproduce the published final seed-0 results: 37.3 s nominal and 45.3 s
noisy/dropout. Source runtime is the integrated version at 5f8e156; later additions
are offline validation, tests and presentation only. Original 10 Hz renders are
repeated at the delivery rate of 15 fps, with a brief hold after arrival; motion
is not interpolated or synthesised. Live overlays show true/estimated pose and
estimation error with +/-3 sigma uncertainty from each recorded trace.

The sensor-validation caption was added after the 4 November offline study.
The five-minute timeline and video SHA-256 are in video-manifest.json. Original
raw captures and local composition tooling are retained in the shared workspace.
The page names the team; actual contributions and AI assistance are declared in
the project README and should be reviewed by each member before submission.

No Canvas submission is performed by this video export. Submit the exact public
project-page URL as instructed; the code and video are linked there together.
