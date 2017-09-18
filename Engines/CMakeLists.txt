SET(targetname Engines)

SET(sources
  MonoEngine.cpp
  FileTracker.cpp
  )

SET(headers
  MonoEngine.h
  FileTracker.h
  )


add_library(${targetname} ${sources} ${headers})


# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
