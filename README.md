How to build MiniTAO
====================

1. Get tools and dependencies.
   * CMake
   * Eigen2 http://eigen.tuxfamily.org/
     (or just "`sudo apt-get install libeigen2-dev`")
   * Google Testing Framework
   * ...maybe others (work in progress)

2. Configure and build.

    mkdir build
    cd build
    cmake ..
    make

3. Test.

    ./tests/testTAO
    ./tests/testMiniTAO


How to run coverage test on MiniTAO
===================================

You need `lcov` from http://ltp.sourceforge.net/coverage/lcov.php
(something like "`sudo apt-get install lcov`" should do the trick).

1. Compile fresh binaries with gcov support.

    cd /path/to/minitao
    mkdir build
    cd build
    cmake -DCOVERAGE=true ..
    make

2. Reset the `lcov` counters.

    cd /path/to/minitao
    lcov --directory build --zerocounters

3. Run the unit tests (and other things that exercise those parts of
   the code that you actually need).

    cd /path/to/minitao/build
    ./tests/testTAO
    ./tests/testMiniTAO

4. Let loose `lcov` and `genhtml` (the latter comes with `lcov`).

    cd /path/to/minitao
    lcov --directory build --capture --output-file cov.info
    genhtml cov.info -o cov/

5. Browse results: open cov/index.html in your favorite browser.
