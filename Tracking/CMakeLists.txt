SET(targetname Tracking)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

SET(sources
  TUMFileTracker.cpp
  ORBFileTracker.cpp
  ARKitFileTracker.cpp
  FileTracker.cpp
  )

SET(headers
  TUMFileTracker.h
  ARKitFileTracker.h
  ORBFileTracker.h
  FileTracker.h
  )


# add_library(${targetname} ${sources} ${headers})

CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
