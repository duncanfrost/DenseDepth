SET(targetname ORUtils)

SET(sources
  FileUtils.cpp
  KeyValueConfig.cpp
  SE3Pose.cpp
  )

SET(headers
  Cholesky.h
  CUDADefines.h
  CUDAPrimitives.h
  FileUtils.h
  HomkerMap.h
  Image.h
  ImageTypes.h
  KeyValueConfig.h
  LexicalCast.h
  MathTypes.h
  MathUtils.h
  Matrix.h
  MemoryBlock.h
  MemoryBlockPersister.h
  MemoryBlockTypes.h
  MemoryDeviceDefs.h
  MEstimators.h
  NVTimer.h
  PixelPrimitives.h
  PlatformIndependence.h
  ProjectionPrimitives.h
  SE3Pose.h
  SVMClassifier.h
  TimeStampedImage.h
  Vector.h
  )

add_library(${targetname} ${sources} ${headers})

# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
