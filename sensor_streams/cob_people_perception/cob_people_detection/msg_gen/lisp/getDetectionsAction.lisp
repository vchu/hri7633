; Auto-generated. Do not edit!


(cl:in-package cob_people_detection-msg)


;//! \htmlinclude getDetectionsAction.msg.html

(cl:defclass <getDetectionsAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type cob_people_detection-msg:getDetectionsActionGoal
    :initform (cl:make-instance 'cob_people_detection-msg:getDetectionsActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type cob_people_detection-msg:getDetectionsActionResult
    :initform (cl:make-instance 'cob_people_detection-msg:getDetectionsActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type cob_people_detection-msg:getDetectionsActionFeedback
    :initform (cl:make-instance 'cob_people_detection-msg:getDetectionsActionFeedback)))
)

(cl:defclass getDetectionsAction (<getDetectionsAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getDetectionsAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getDetectionsAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_people_detection-msg:<getDetectionsAction> is deprecated: use cob_people_detection-msg:getDetectionsAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <getDetectionsAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_people_detection-msg:action_goal-val is deprecated.  Use cob_people_detection-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <getDetectionsAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_people_detection-msg:action_result-val is deprecated.  Use cob_people_detection-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <getDetectionsAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_people_detection-msg:action_feedback-val is deprecated.  Use cob_people_detection-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getDetectionsAction>) ostream)
  "Serializes a message object of type '<getDetectionsAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getDetectionsAction>) istream)
  "Deserializes a message object of type '<getDetectionsAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getDetectionsAction>)))
  "Returns string type for a message object of type '<getDetectionsAction>"
  "cob_people_detection/getDetectionsAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getDetectionsAction)))
  "Returns string type for a message object of type 'getDetectionsAction"
  "cob_people_detection/getDetectionsAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getDetectionsAction>)))
  "Returns md5sum for a message object of type '<getDetectionsAction>"
  "c5731045e4ba27f1248635d37b384b4e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getDetectionsAction)))
  "Returns md5sum for a message object of type 'getDetectionsAction"
  "c5731045e4ba27f1248635d37b384b4e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getDetectionsAction>)))
  "Returns full string definition for message of type '<getDetectionsAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%getDetectionsActionGoal action_goal~%getDetectionsActionResult action_result~%getDetectionsActionFeedback action_feedback~%~%================================================================================~%MSG: cob_people_detection/getDetectionsActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%getDetectionsGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: cob_people_detection/getDetectionsGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal~%float32 maximum_message_age		# maximal allowed age of response message (in s), 0 means we do not care~%float32 timeout				# maximal waiting time (in s) for receiving a detection, 0 is infinitely. If the detection pipeline was not running before, it will start upon receiving this message and collect messages for the full timeout period.~%~%================================================================================~%MSG: cob_people_detection/getDetectionsActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%getDetectionsResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: cob_people_detection/getDetectionsResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result~%cob_people_detection_msgs/DetectionArray detections	# array of detections~%~%================================================================================~%MSG: cob_people_detection_msgs/DetectionArray~%Header header~%Detection[] detections~%~%================================================================================~%MSG: cob_people_detection_msgs/Detection~%Header header~%string label~%string detector~%float32 score~%Mask mask~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: cob_people_detection_msgs/Mask~%# this message is used to mark where an object is present in an image~%# this can be done either by a roi region on the image (less precise) or a mask (more precise)~%~%Rect roi~%~%# in the case when mask is used, 'roi' specifies the image region and 'mask'~%# (which should be of the same size) a binary mask in that region~%sensor_msgs/Image mask~%~%# in the case there is 3D data available, 'indices' are used to index the ~%# part of the point cloud representing the object~%#pcl/PointIndices indices~%~%================================================================================~%MSG: cob_people_detection_msgs/Rect~%int32 x~%int32 y~%int32 width~%int32 height~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: cob_people_detection/getDetectionsActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%getDetectionsFeedback feedback~%~%================================================================================~%MSG: cob_people_detection/getDetectionsFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getDetectionsAction)))
  "Returns full string definition for message of type 'getDetectionsAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%getDetectionsActionGoal action_goal~%getDetectionsActionResult action_result~%getDetectionsActionFeedback action_feedback~%~%================================================================================~%MSG: cob_people_detection/getDetectionsActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%getDetectionsGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: cob_people_detection/getDetectionsGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal~%float32 maximum_message_age		# maximal allowed age of response message (in s), 0 means we do not care~%float32 timeout				# maximal waiting time (in s) for receiving a detection, 0 is infinitely. If the detection pipeline was not running before, it will start upon receiving this message and collect messages for the full timeout period.~%~%================================================================================~%MSG: cob_people_detection/getDetectionsActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%getDetectionsResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: cob_people_detection/getDetectionsResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result~%cob_people_detection_msgs/DetectionArray detections	# array of detections~%~%================================================================================~%MSG: cob_people_detection_msgs/DetectionArray~%Header header~%Detection[] detections~%~%================================================================================~%MSG: cob_people_detection_msgs/Detection~%Header header~%string label~%string detector~%float32 score~%Mask mask~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: cob_people_detection_msgs/Mask~%# this message is used to mark where an object is present in an image~%# this can be done either by a roi region on the image (less precise) or a mask (more precise)~%~%Rect roi~%~%# in the case when mask is used, 'roi' specifies the image region and 'mask'~%# (which should be of the same size) a binary mask in that region~%sensor_msgs/Image mask~%~%# in the case there is 3D data available, 'indices' are used to index the ~%# part of the point cloud representing the object~%#pcl/PointIndices indices~%~%================================================================================~%MSG: cob_people_detection_msgs/Rect~%int32 x~%int32 y~%int32 width~%int32 height~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: cob_people_detection/getDetectionsActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%getDetectionsFeedback feedback~%~%================================================================================~%MSG: cob_people_detection/getDetectionsFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getDetectionsAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getDetectionsAction>))
  "Converts a ROS message object to a list"
  (cl:list 'getDetectionsAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
