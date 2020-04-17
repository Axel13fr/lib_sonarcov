catkin_add_gtest(swathRecord_test ./swathRecord_test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
target_link_libraries(swathRecord_test sonarcov ${catkin_LIBRARIES} )

catkin_add_gtest(swathRecorder_test ./swathRecorder_test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
target_link_libraries(swathRecorder_test sonarcov ${catkin_LIBRARIES} )

catkin_add_gtest(pathPlan_test ./pathPlan_test.cpp ./testlogger.h WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
target_link_libraries(pathPlan_test sonarcov ${catkin_LIBRARIES} )


# So that the python file shows up in IDE
FILE(GLOB_RECURSE OTHER_FILES
    "./*.py"
)

add_custom_target(other_files SOURCES ${OTHER_FILES})
