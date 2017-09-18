SET(targetname GUI)

SET(sources
  ARWindow.cpp
  Button.cpp
  glTexture.cpp
  glTools.cpp
  # MapWindow.cpp
  VisualisationModule.cpp
  Window.cpp
  )

SET(headers
  ARWindow.h
  Button.h
  glTexture.h
  glTools.h
  # MapWindow.h
  VisualisationModule.h
  Window.h
  )


add_library(${targetname} ${sources} ${headers})


#l Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
