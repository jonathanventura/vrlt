# - Try to find Sophus lib

if (SOPHUS_INCLUDE_DIR)

else (SOPHUS_INCLUDE_DIR)

  find_path(SOPHUS_INCLUDE_DIR NAMES sophus
      PATHS
      ${CMAKE_INSTALL_PREFIX}/include
      #add path to local build, if build form src
    )

endif(SOPHUS_INCLUDE_DIR)

