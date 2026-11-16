"""M12 camera following: the code that runs on the Raspberry Pi Zero 2 W.

This package imports only numpy, scipy and pupil-apriltags (no MuJoCo, no simulator), so
it installs on Raspberry Pi OS as it is. The simulation runs the same code: each rendered
frame goes through pipeline.FramePipeline (the phase C tracker, the pose fit through the
lens model, the 74 deg view gate and the report). Only the mount differs: the simulator
converts with the plant's own camera pose, the Pi with FixedMount and the body's lean.
FixedMount itself is checked by the host tests, against the simulator's conversion.

What is not here yet (phase E): the camera capture (picamera2), the link to the Teensy
(the bus is a phase E choice), the lens calibration and the body lean the Teensy would send.
"""
