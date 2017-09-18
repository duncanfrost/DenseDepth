add_executable(main main.cpp)

message("INTERN LIBS:")
message("${INTERN_LIBS}")

target_link_libraries(main ${INTERN_LIBS} ${EXTERN_LIBS})
