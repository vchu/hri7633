
(cl:in-package :asdf)

(defsystem "cob_people_detection_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ColorDepthImage" :depends-on ("_package_ColorDepthImage"))
    (:file "_package_ColorDepthImage" :depends-on ("_package"))
    (:file "Mask" :depends-on ("_package_Mask"))
    (:file "_package_Mask" :depends-on ("_package"))
    (:file "DetectionArray" :depends-on ("_package_DetectionArray"))
    (:file "_package_DetectionArray" :depends-on ("_package"))
    (:file "ColorDepthImageArray" :depends-on ("_package_ColorDepthImageArray"))
    (:file "_package_ColorDepthImageArray" :depends-on ("_package"))
    (:file "Rect" :depends-on ("_package_Rect"))
    (:file "_package_Rect" :depends-on ("_package"))
    (:file "Detection" :depends-on ("_package_Detection"))
    (:file "_package_Detection" :depends-on ("_package"))
  ))