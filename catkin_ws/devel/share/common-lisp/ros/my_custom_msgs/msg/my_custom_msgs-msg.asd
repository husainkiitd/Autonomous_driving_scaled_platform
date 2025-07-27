
(cl:in-package :asdf)

(defsystem "my_custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PathArray" :depends-on ("_package_PathArray"))
    (:file "_package_PathArray" :depends-on ("_package"))
  ))