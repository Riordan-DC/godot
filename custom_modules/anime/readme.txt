TODO
0. ozzgd run update in its own process loop
1. signal for script animation update


PITFALLS
Explicit namespacing. I spent like 4 hours debugging a link error in ozz/utils.h.
The definition for MultiplySoATransformQuaternion could not be found. Yet classes
in the same file could be? In the end what solved this was explicitly making the
function signature ozz::sample::MultiplySoATransformQuaternion. "using" was already
in use but it did not seem to work.