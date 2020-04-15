catkin_add_gtest(swathRecord_test ./swathRecord_test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
target_link_libraries(swathRecord_test lib_sonarcov ${catkin_LIBRARIES} )

