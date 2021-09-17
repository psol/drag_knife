# drag_knife
G-code post processor for drag knife (such as Donek) on a CNC.

A drag knife cannot take sharp turns. The post-processor adds an offset and
gently turns the blade in the G-code whenever it detects a sharp turn.
This is developed for (and tested against) [Lightburn](https://lightburnsoftware.com)
files.

There are a number of post-processors available but they typically don't work on macOS.
I wanted a lightweight solution that would complement Lightburn while waiting for it
to (implement drag knife support)[https://lightburn.fider.io/posts/1388/lightburn-to-support-drag-knives-and-drag-engravers].

Be warned that I am no expert in G-code and that this code is still in very early
phase. It probably does not handle all the cases and all the files.

The code is specifically targeting Lightburn files with the GBRL post-processor,
for example it only deals with the ; comment because Lightburn inserts no other comments.

For obvious reasons, you should use Line mode only. I recommend a lead-in (in the advanced
layer settings) such that the knife can adjust it's entry angle.

The code only processes G0 (rapid linear motion) and G1 (cutting motion).
Although it inserts G2 and G3 in the post-processed file, it does not process
them in the input file.

The knife is raised before the turn but it cannot be raised out of the material or
it would not turn. These light cuts are usually out of the part being cut...
but not always. Donek recommends to cut from the back side so the light cuts are
not visible in the finished part.
Improved inside/outside detection would detect those and move the knife to scrap material
for the turn.

I may or may not address these restrictions in a future version.

Last but not least, it's Python 2 because macOS ships with Python 2.7 by default
(and I don't want to go to the trouble of installing Python 3 on all my machines
for such a small piece of code).

In a nutshell, the logic is:

- read the G-code sequentially, keeping track of the last 3 coordinates of the cutter;
  3 coordinates define 2 moves: the previous move and the one being post-processed
- recognise linear motion (G0, G1)
    - G0: raise the knife before the move, lower it after, reset the buffer of position
    - G1: compute the angle between the last move and the current one
        - if the angle is sharp, raise and turn the knife before executing the move
        - otherwise, just execute the move
- there's special logic to handle sharp angle on moves too short for the knife to turn;
  the post-processor waits until the knife will travel its minimal distance before
  processing the turn