add_executable(rbegin rbegin.cpp)
target_link_libraries(rbegin ${LIBS})


add_executable(vector_assign vector_assign.cpp)
target_link_libraries(vector_assign ${LIBS})


add_executable(vector_ptr vector_ptr.cpp)
target_link_libraries(vector_ptr ${LIBS})

add_executable(vector_ vector.cpp)
target_link_libraries(vector_ ${LIBS})

add_executable(map_ map.cpp)
target_link_libraries(map_ ${LIBS})

add_executable(map_vector_pair map_vector_pair.cpp)
target_link_libraries(map_vector_pair ${LIBS})