vrlt
====

Visual Reconstruction and Localization Toolkit

====
Dependencies:
- Eigen3 (can be installed with the macports package manager)
- Sophus (build from the current github master: https://github.com/strasdat/Sophus; requires Eigen3)
- ceres-solver 1.10 (using the installation of macports fails, due to a known issue of the ceres-solver and Eigen3: https://groups.google.com/forum/#!msg/ceres-solver/o8W95uEjOGQ/JqLlxaPTpjoJ ; the solution is to install it using the macports libs: port install -s ceres-solver)
- geographiclib 1.36 (macports installation does the job)
- OpenCV 3.0 beta: (openCV 3.0 is required since the LineSegmentDescriptor used in VRLT is not implemented in the stable openCV 2.4 version; openCV 3.0 needs to be build form sources WITH the module extension xfeatures2d form the contribution repo: https://github.com/itseez/opencv_contrib/ , because the SIFT feature extraction moved in version 3.0 from the nonfree to the external xfeatrues2d module)
