# vrlt #

Visual Reconstruction and Localization Toolkit

Jonathan Ventura

Assistant Professor

University of Colorado Colorado Springs

http://jventura.net/

This software was developed for my 2012 Ph.D. dissertation and later extended and improved.  If you use it, please cite the following work:

Ventura, J., and T. Höllerer, "Wide-Area Scene Mapping for Mobile Visual Tracking", IEEE International Symposium on Mixed and Augmented Reality.

## Dependencies ##

Under OS X, we recommend installing dependencies with Homebrew.

- Eigen (brew install eigen)
- Sophus (build from the current github master: https://github.com/strasdat/Sophus ; requires Eigen3)
- Ceres (brew install ceres-solver)
- geographiclib (brew install geographiclib)
- OpenCV 3 (brew install opencv3 --with-nonfree --with-contrib --without-eigen)
OpenCV version 3 is needed for the LSD line detector. The `--without-eigen` flag is necessary to avoid a conflict with Ceres.  The flags `--with-non-free` and `--with-contrib` are needed for SIFT.  Homebrew refuses to link opencv3, but the CMakeLists.txt is setup to find it at the Homebrew installation location `/usr/local/opt/opencv3/share/OpenCV`.

## Compilation ##

Follow the typical process for building with CMake:

    mkdir build
    cd build
    cmake ..
    make

## Testing ##

The wiki contains tutorial documents for how to run the reconstruction pipeline and tracker.

A test dataset <a href="https://www.dropbox.com/s/368ggcc65dk0yx6/VillageDataset.tgz?dl=0">available here</a> (about 400 MB).
