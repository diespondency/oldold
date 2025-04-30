file(GLOB_RECURSE SOURCES "*.cc" "*.c")
add_library(nrscope_libs STATIC ${SOURCES})

# message(${PROJECT_SOURCE_DIR})
# target_include_directories(ngscope_dci PUBLIC ${PROJECT_SOURCE_DIR}/ngscope/hdr)

if(ENABLE_GUI AND SRSGUI_FOUND)
  target_link_libraries(nrscope_libs ${SRSGUI_LIBRARIES})

target_link_libraries(nrscope_libs ${SRSRAN_SOURCES}
                                   ${SRSUE_SOURCES})
endif()

if (LIBASN5G)
  target_link_libraries(nrscope_libs ${LIBASN5G_LIBRARY})
endif(LIBASN5G)
