FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cob_people_detection/msg"
  "../src/cob_people_detection/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/recognitionTrigger.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_recognitionTrigger.lisp"
  "../srv_gen/lisp/finishRecording.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_finishRecording.lisp"
  "../srv_gen/lisp/captureImage.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_captureImage.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
