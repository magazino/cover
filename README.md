# COVER

## Idea
The library splits a given polygon into a inscribed inner polygon and the
remainders.
This allows you to check only the remainders cell-wise for collision and the
inscribed area for inscribed cost.

![schematic](doc/schematic.svg)

Have a look at the potato above.
The red shape shows the entire polygon.
The green area shows the inscribed sub-area.
There its sufficient to check for inscribed costs **on** the dashed line.
The cells inside the red areas one needs to check all cells.

## Using

The main interface for ros-users lies in cover_ros.hpp.
There you will find a function to generate a footprint from a polygon and
functions for checking this footprint

## Debugging

If you want to visualize the generated footprint, enable the debug output
for this code.
Then we will output [well known text representation](https://en.wikipedia.org/wiki/Well-known_text_representation_of_geometry) for every polygon.
You can use one of the online [wkt-viewers](http://dev.openlayers.org/examples/vector-formats.html) to visualize the result
