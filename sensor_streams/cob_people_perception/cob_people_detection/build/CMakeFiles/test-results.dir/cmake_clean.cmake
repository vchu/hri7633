FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cob_people_detection/msg"
  "../src/cob_people_detection/srv"
  "CMakeFiles/test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
