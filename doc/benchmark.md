# Benchmarks

The benchmarks below were generated on a AMD Ryzen 5 PRO 4650U with Radeon Graphics.
Typically the prefix is describes the test-type (for example to_outline).
The postfix cover denotes that this is the a function from this library.
The postfix costmap indicates that this is a comparable function from the
[costmap_2d](http://wiki.ros.org/costmap_2d/layered) (or 
[base_local_planner](http://wiki.ros.org/base_local_planner)) library.
The postfix nav_grid_iterators indicates that this is a comparable function from
the
[nav_grid_iterators](https://github.com/locusrobotics/robot_navigation/tree/noetic/nav_grid_iterators)
library.
The last number (0 to 5) is the test-parameter (the tests have been done of different polygonal footprints).

Note: The footprints 0 to 3 are concave.
Since the ros-library is not suited for concave footprints they have been
omitted form the to_area_costmap benchmark.


|Benchmark                              |         Time|             CPU|   Iterations|
|---------------------------------------|-------------|----------------|-------------|
|to_line_cover                          |       161 ns|          160 ns|      4041687|
|to_line_costmap                        |       544 ns|          544 ns|      1293362|
|costmap_fixture/to_outline_cover/0             |        280 ns|          280 ns|      2545658|
|costmap_fixture/to_outline_cover/1             |        370 ns|          369 ns|      1836983|
|costmap_fixture/to_outline_cover/2             |        466 ns|          466 ns|      1461823|
|costmap_fixture/to_outline_cover/3             |        882 ns|          882 ns|       784333|
|costmap_fixture/to_outline_cover/4             |        321 ns|          321 ns|      2162648|
|costmap_fixture/to_outline_cover/5             |        346 ns|          345 ns|      2011996|
|costmap_fixture/to_outline_costmap/0           |        803 ns|          803 ns|       869883|
|costmap_fixture/to_outline_costmap/1           |       1227 ns|         1226 ns|       568397|
|costmap_fixture/to_outline_costmap/2           |       1436 ns|         1434 ns|       494009|
|costmap_fixture/to_outline_costmap/3           |        764 ns|          764 ns|       921903|
|costmap_fixture/to_outline_costmap/4           |        896 ns|          895 ns|       785647|
|costmap_fixture/to_outline_costmap/5           |        785 ns|          785 ns|       890924|
|costmap_fixture/to_outline_nav_grid_iterators/0|      11038 ns|        11031 ns|        63569|
|costmap_fixture/to_outline_nav_grid_iterators/1|      16408 ns|        16396 ns|        42674|
|costmap_fixture/to_outline_nav_grid_iterators/2|      19945 ns|        19930 ns|        35209|
|costmap_fixture/to_outline_nav_grid_iterators/3|      32062 ns|        32049 ns|        21784|
|costmap_fixture/to_outline_nav_grid_iterators/4|       9161 ns|         9154 ns|        76708|
|costmap_fixture/to_outline_nav_grid_iterators/5|       6650 ns|         6645 ns|       105647|
|costmap_fixture/to_area_cover/0                |       9147 ns|         9140 ns|        77556|
|costmap_fixture/to_area_cover/1                |       9819 ns|         9810 ns|        71358|
|costmap_fixture/to_area_cover/2                |      13382 ns|        13376 ns|        51419|
|costmap_fixture/to_area_cover/3                |      26164 ns|        26152 ns|        26701|
|costmap_fixture/to_area_cover/4                |       2315 ns|         2314 ns|       299387|
|costmap_fixture/to_area_cover/5                |       1004 ns|         1003 ns|       687788|
|costmap_fixture/to_area_costmap/4              |      35332 ns|        35289 ns|        19294|
|costmap_fixture/to_area_costmap/5              |       8367 ns|         8358 ns|        83861|
|costmap_fixture/to_area_nav_grid_iterators/0   |     137955 ns|       137889 ns|         5088|
|costmap_fixture/to_area_nav_grid_iterators/1   |     148128 ns|       148045 ns|         4758|
|costmap_fixture/to_area_nav_grid_iterators/2   |     179057 ns|       178860 ns|         3818|
|costmap_fixture/to_area_nav_grid_iterators/3   |     919363 ns|       918951 ns|          767|
|costmap_fixture/to_area_nav_grid_iterators/4   |      49982 ns|        49965 ns|        13973|
|costmap_fixture/to_area_nav_grid_iterators/5   |      18011 ns|        17995 ns|        37895|


Below is the benchmark for the splitting of the footprint (continuous_footprint_generator).
The generation is with ~800 microseconds way slower than a one time check of the same footprints
(see above to_area_cover/4 and to_area_cover/5).
These cost could be however amortized, if the generated representation can be
cached. The checks is_free_fixture/continuous_footprint are slightly faster than
their counter part is_free_fixture/area_generator.
The discrete checks is_free_fixture/discrete_footprint offer a clear advantage
over the "traditional" checks is_free_fixture/discrete_polygon.

|Benchmark                              |         Time|             CPU|   Iterations|
|---------------------------------------|-------------|----------------|-------------|
|continuous_footprint_generator/4       |    144196 ns|       143942 ns|         4692|
|continuous_footprint_generator/5       |    779311 ns|       777771 ns|          890|
|is_free_fixture/area_generator/4       |     30192 ns|        30149 ns|        23153|
|is_free_fixture/area_generator/5       |      9920 ns|         9913 ns|        70259|
|is_free_fixture/discrete_polygon/4     |     18452 ns|        18430 ns|        35501|
|is_free_fixture/discrete_polygon/5     |      4840 ns|         4834 ns|       152498|
|is_free_fixture/continuous_footprint/4 |     20164 ns|        20139 ns|        35150|
|is_free_fixture/continuous_footprint/5 |      9310 ns|         9299 ns|        71894|
|is_free_fixture/discrete_footprint/4   |      3164 ns|         3161 ns|       220604|
|is_free_fixture/discrete_footprint/5   |       696 ns|          695 ns|      1030751|
