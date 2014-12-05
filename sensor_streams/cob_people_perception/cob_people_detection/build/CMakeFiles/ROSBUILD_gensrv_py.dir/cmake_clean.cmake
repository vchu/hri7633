FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cob_people_detection/msg"
  "../src/cob_people_detection/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/cob_people_detection/srv/__init__.py"
  "../src/cob_people_detection/srv/_recognitionTrigger.py"
  "../src/cob_people_detection/srv/_finishRecording.py"
  "../src/cob_people_detection/srv/_captureImage.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
