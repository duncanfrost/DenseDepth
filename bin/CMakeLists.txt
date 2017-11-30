INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)

# add_executable(main main.cpp)

CUDA_ADD_EXECUTABLE(main main.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(fusion fusion.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(tum2desk tum2desk.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(tum2xyz tum2xyz.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(tum3 tum3.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(tumcli tumcli.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(smooth smooth.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})
CUDA_ADD_EXECUTABLE(arkit arkit.cpp  OPTIONS --generate-code arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY})

target_link_libraries(main ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(fusion ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(tum2desk ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(tum2xyz ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(tum3 ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(tumcli ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(smooth ${INTERN_LIBS} ${EXTERN_LIBS})
target_link_libraries(arkit ${INTERN_LIBS} ${EXTERN_LIBS})

configure_file(../data/noisyLena.png noisyLena.png COPYONLY)
