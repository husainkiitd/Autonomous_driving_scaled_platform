;; Auto-generated. Do not edit!


(when (boundp 'my_custom_msgs::PathArray)
  (if (not (find-package "MY_CUSTOM_MSGS"))
    (make-package "MY_CUSTOM_MSGS"))
  (shadow 'PathArray (find-package "MY_CUSTOM_MSGS")))
(unless (find-package "MY_CUSTOM_MSGS::PATHARRAY")
  (make-package "MY_CUSTOM_MSGS::PATHARRAY"))

(in-package "ROS")
;;//! \htmlinclude PathArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass my_custom_msgs::PathArray
  :super ros::object
  :slots (_poses ))

(defmethod my_custom_msgs::PathArray
  (:init
   (&key
    ((:poses __poses) ())
    )
   (send-super :init)
   (setq _poses __poses)
   self)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get my_custom_msgs::PathArray :md5sum-) "db8ce58a1deaf0d1f2e7fdd257963e81")
(setf (get my_custom_msgs::PathArray :datatype-) "my_custom_msgs/PathArray")
(setf (get my_custom_msgs::PathArray :definition-)
      "geometry_msgs/Pose[] poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :my_custom_msgs/PathArray "db8ce58a1deaf0d1f2e7fdd257963e81")


