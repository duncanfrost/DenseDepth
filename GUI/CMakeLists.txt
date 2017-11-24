SET(targetname GUI)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

SET(sources
  DepthWindow.cpp
  ARWindow.cpp
  Button.cpp
  glTexture.cpp
  glTools.cpp
  MapWindow.cpp
  FusionWindow.cpp
  VisualisationModule.cpp
  Window.cpp
  )

SET(headers
  DepthWindow.h
  ARWindow.h
  Button.h
  glTexture.h
  glTools.h
  MapWindow.h
  FusionWindow.h
  VisualisationModule.h
  Window.h
  )


# add_library(${targetname} ${sources} ${headers})

CUDA_ADD_LIBRARY(${targetname} STATIC ${sources} ${headers} ${templates} OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

#l Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
