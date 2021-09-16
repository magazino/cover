^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cover
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2021-09-16)
------------------
* Merged in SW-49813_stl_area_iterator (pull request #12)
  SW-49813 stl area iterator
  * SW-48772 Rewrite scan line algorithm to include boundary by construction
  * SW-48772 Write comments in a more clear words
  * SW-48772 Shift to ranges
  * SW-48772 Catch cases where all points are colinear
  * SW-48772 Add missing include
  * SW-48772 Format test
  * SW-48772 Change from array to ints
  * SW-49813 Added stl compatible fill area iterator
  * SW-49813 Merge with master
  * SW-49813 Changes for PR
  Approved-by: Dmitrij Dorezyuk
* fixup compile warning for tests
* fixup the pedantic warnings for ros-includes
* update maintainer
* Merged in SW-48772_create_area_iterator (pull request #11)
  SW-48772 Rewrite scan line algorithm to include boundary by construction
  Approved-by: Dmitrij Dorezyuk
* Merged in SW-48771_Add_non_convex_support (pull request #10)
  SW-48771 Add non convex support
  Approved-by: Dmitrij Dorezyuk
* Merged in dima/area (pull request #9)

0.3.0 (2020-10-14)
------------------
* fix bug
* use discrete_polygon
* update documentation
* disable clang-tidy by default
* fix tests
* port to noetic

0.2.0 (2020-08-14)
------------------
* document interfaces
* use own raytrace function
* fix missing size-check
* fix raytrace and add tests
* add raytrace code

0.1.0 (2020-07-31)
------------------
* fixup missing clang-tidy dependency
* adjust doc
* extend readme on how to debug the code
* add debugging prints
* fix readme non-sense
* update doc
* reuse test-utils for perf testing
* dont polute geom::geos namespace
* document the api
* fix make_circle
* rename signed_diff to signed_y_diff
* rename scan_line to line_scan
* fixup clang-tidy issues
* formatting
* add test for unit circle
* add benchmarks
* cleanup
* add test
* fix unsigned underflow
* update doc
* adjust documentation
* fix area fill algorithm
* drop costmaps fill-poly algorithm\
* adjust cost
* add doc
* fix bugs
* add first ros-tests
* fix tests
* add tests
* prototype
* init
