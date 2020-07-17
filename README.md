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