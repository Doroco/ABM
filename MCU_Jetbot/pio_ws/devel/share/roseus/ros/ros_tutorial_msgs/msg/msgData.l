;; Auto-generated. Do not edit!


(when (boundp 'ros_tutorial_msgs::msgData)
  (if (not (find-package "ROS_TUTORIAL_MSGS"))
    (make-package "ROS_TUTORIAL_MSGS"))
  (shadow 'msgData (find-package "ROS_TUTORIAL_MSGS")))
(unless (find-package "ROS_TUTORIAL_MSGS::MSGDATA")
  (make-package "ROS_TUTORIAL_MSGS::MSGDATA"))

(in-package "ROS")
;;//! \htmlinclude msgData.msg.html


(defclass ros_tutorial_msgs::msgData
  :super ros::object
  :slots (_angle _mode ))

(defmethod ros_tutorial_msgs::msgData
  (:init
   (&key
    ((:angle __angle) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _angle __angle)
   (setq _mode (round __mode))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; float64[3] _angle
    (* 8    3)
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _angle
     (dotimes (i 3)
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _angle
   (dotimes (i (length _angle))
     (setf (elt _angle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ros_tutorial_msgs::msgData :md5sum-) "1b7cf6a5d1b828728f05b43a5c9cccb2")
(setf (get ros_tutorial_msgs::msgData :datatype-) "ros_tutorial_msgs/msgData")
(setf (get ros_tutorial_msgs::msgData :definition-)
      "float64[3] angle
uint8 mode


")



(provide :ros_tutorial_msgs/msgData "1b7cf6a5d1b828728f05b43a5c9cccb2")


