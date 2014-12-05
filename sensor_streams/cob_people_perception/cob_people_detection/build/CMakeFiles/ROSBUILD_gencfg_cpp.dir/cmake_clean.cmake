FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cob_people_detection/msg"
  "../src/cob_people_detection/srv"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/cob_people_detection/sensor_message_gatewayConfig.h"
  "../docs/sensor_message_gatewayConfig.dox"
  "../docs/sensor_message_gatewayConfig-usage.dox"
  "../src/cob_people_detection/cfg/sensor_message_gatewayConfig.py"
  "../docs/sensor_message_gatewayConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
