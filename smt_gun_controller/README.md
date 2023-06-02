# SMT Gun Controller

This is the gun controller on the physical hardware. It receives messages on the `/gun_cmd` topic which specify
an angle for the height of the gun and then proceeds to move the gun to the specified angle and fire at it,
finally moving the gun back to the original level.
