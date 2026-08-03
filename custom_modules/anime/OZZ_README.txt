todo
----
use Ozz blend job to interpolate skeleton state

This means I have to do ltm job after interpolation, not before.

Instead of calling 
    . update_skeleton_interpolated() I need to call Ozz.update_skeleton_interpolated(state, delta)

    To create a skeleton state you should call Ozz.create_state()
    . Dont use apply_animation_state

    while testing dont call update_skeleton()



workflow

ozz.play_animation()
~ozz.blend_animation()
ozz.apply_animation_state(skeleton_state, animation_state)
ozz.update_skeleton_interpolated()