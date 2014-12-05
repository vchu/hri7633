FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/cob_people_detection_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cob_people_detection_msgs/msg/__init__.py"
  "../src/cob_people_detection_msgs/msg/_ColorDepthImage.py"
  "../src/cob_people_detection_msgs/msg/_Mask.py"
  "../src/cob_people_detection_msgs/msg/_DetectionArray.py"
  "../src/cob_people_detection_msgs/msg/_ColorDepthImageArray.py"
  "../src/cob_people_detection_msgs/msg/_Rect.py"
  "../src/cob_people_detection_msgs/msg/_Detection.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
