INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

# add_executable(main main.cpp)

CUDA_ADD_EXECUTABLE(main main.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

CUDA_ADD_EXECUTABLE(fusion fusion.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

target_link_libraries(main ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(fusion ${INTERN_LIBS} ${EXTERN_LIBS})
