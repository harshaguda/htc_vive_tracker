# create an example application
ADD_EXECUTABLE(htc_vive_tracker_test htc_vive_tracker_test.cpp)
# link necessary libraries
TARGET_LINK_LIBRARIES(htc_vive_tracker_test htc_vive_tracker)

# create controller relative to tracker example
ADD_EXECUTABLE(controller_relative_to_tracker controller_relative_to_tracker.cpp)
# link necessary libraries
TARGET_LINK_LIBRARIES(controller_relative_to_tracker htc_vive_tracker)
