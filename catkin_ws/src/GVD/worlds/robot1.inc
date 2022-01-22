define topurg ranger
(
    sensor(
        range [0.0 8.0]
        fov 360
        samples 1440
    )
    # generic block properties
    color_rgba [ 1 0 0 0.5]
    size [0.050 0.050 0.100]
)

define robot1 position
(
    size [0.50 0.50 0.250]
    origin [0 0.000 0.000 0.000]
    gui_nose 1
    drive "diff"
    topurg(pose [0.0 0.000 0.000 0.000])
)