#
#   humanoid library
# 

#for eigen3
find_package(Eigen3 REQUIRED)

# for plot
find_package(PythonLibs 2.7 REQUIRED)

set(HUMANOID_INCLUDE_DIRS /usr/local/include/humanoid ${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
set(HUMANOID_LIBRARIES /usr/local/lib/libhumanoid.so ${PYTHON_LIBRARIES})
