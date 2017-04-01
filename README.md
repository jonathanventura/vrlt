vrlt
====

Visual Reconstruction and Localization Toolkit

====
Under OS X, we recommend installing dependencies with Homebrew.

Dependencies:
- Eigen (brew install eigen)
- Sophus (build from the current github master: https://github.com/strasdat/Sophus ; requires Eigen3)
- Ceres (brew install ceres-solver)
- geographiclib (brew install geographiclib)
- OpenCV 3 (brew install opencv3 --with-nonfree --with-contrib --without-eigen)
OpenCV version 3 is needed for the LSD line detector. The `--without-eigen` flag is necessary to avoid a conflict with Ceres.  The flags `--with-non-free` and `--with-contrib` are needed for SIFT.  Homebrew refuses to link opencv3, but the CMakeLists.txt is setup to find it at the Homebrew installation location `/usr/local/opt/opencv3/share/OpenCV`.

===
Test dataset <a href="https://www.dropbox.com/s/368ggcc65dk0yx6/VillageDataset.tgz?dl=0">available here</a> (about 400 MB).
