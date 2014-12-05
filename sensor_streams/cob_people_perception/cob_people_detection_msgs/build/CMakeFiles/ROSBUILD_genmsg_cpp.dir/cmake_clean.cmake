FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/cob_people_detection_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/cob_people_detection_msgs/ColorDepthImage.h"
  "../msg_gen/cpp/include/cob_people_detection_msgs/Mask.h"
  "../msg_gen/cpp/include/cob_people_detection_msgs/DetectionArray.h"
  "../msg_gen/cpp/include/cob_people_detection_msgs/ColorDepthImageArray.h"
  "../msg_gen/cpp/include/cob_people_detection_msgs/Rect.h"
  "../msg_gen/cpp/include/cob_people_detection_msgs/Detection.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
