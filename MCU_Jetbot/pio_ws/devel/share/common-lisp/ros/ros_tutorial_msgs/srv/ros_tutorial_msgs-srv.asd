
(cl:in-package :asdf)

(defsystem "ros_tutorial_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srvTest" :depends-on ("_package_srvTest"))
    (:file "_package_srvTest" :depends-on ("_package"))
  ))