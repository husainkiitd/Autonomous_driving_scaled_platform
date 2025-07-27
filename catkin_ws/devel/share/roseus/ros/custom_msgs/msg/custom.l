;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::custom)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'custom (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::CUSTOM")
  (make-package "CUSTOM_MSGS::CUSTOM"))

(in-package "ROS")
;;//! \htmlinclude custom.msg.html


(defclass custom_msgs::custom
  :super ros::object
  :slots (_x _y _theta ))

(defmethod custom_msgs::custom
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0 :element-type :integer))
    ((:y __y) (make-array 0 :initial-element 0 :element-type :integer))
    ((:theta __theta) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _theta __theta)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; int32[] _x
    (* 4    (length _x)) 4
    ;; int32[] _y
    (* 4    (length _y)) 4
    ;; int32[] _theta
    (* 4    (length _theta)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (write-long (elt _x i) s)
       )
     ;; int32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (write-long (elt _y i) s)
       )
     ;; int32[] _theta
     (write-long (length _theta) s)
     (dotimes (i (length _theta))
       (write-long (elt _theta i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _theta
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _theta (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _theta i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get custom_msgs::custom :md5sum-) "df8fc9de899ddde5dfec399a5e112d32")
(setf (get custom_msgs::custom :datatype-) "custom_msgs/custom")
(setf (get custom_msgs::custom :definition-)
      "int32[] x
int32[] y
int32[] theta

")



(provide :custom_msgs/custom "df8fc9de899ddde5dfec399a5e112d32")


