SET(targetname ImageSource)

SET(sources
  PhoneSource.cpp
  )

SET(headers
  PhoneSource.h
  )


add_library(${targetname} ${sources} ${headers})


# Add this library to our internal libraries
set(INTERN_LIBS ${INTERN_LIBS} ${targetname} PARENT_SCOPE)
