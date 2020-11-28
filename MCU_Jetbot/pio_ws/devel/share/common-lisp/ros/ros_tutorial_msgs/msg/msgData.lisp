; Auto-generated. Do not edit!


(cl:in-package ros_tutorial_msgs-msg)


;//! \htmlinclude msgData.msg.html

(cl:defclass <msgData> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msgData (<msgData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_tutorial_msgs-msg:<msgData> is deprecated: use ros_tutorial_msgs-msg:msgData instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <msgData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial_msgs-msg:angle-val is deprecated.  Use ros_tutorial_msgs-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <msgData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial_msgs-msg:mode-val is deprecated.  Use ros_tutorial_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgData>) ostream)
  "Serializes a message object of type '<msgData>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'angle))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgData>) istream)
  "Deserializes a message object of type '<msgData>"
  (cl:setf (cl:slot-value msg 'angle) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'angle)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgData>)))
  "Returns string type for a message object of type '<msgData>"
  "ros_tutorial_msgs/msgData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgData)))
  "Returns string type for a message object of type 'msgData"
  "ros_tutorial_msgs/msgData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgData>)))
  "Returns md5sum for a message object of type '<msgData>"
  "1b7cf6a5d1b828728f05b43a5c9cccb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgData)))
  "Returns md5sum for a message object of type 'msgData"
  "1b7cf6a5d1b828728f05b43a5c9cccb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgData>)))
  "Returns full string definition for message of type '<msgData>"
  (cl:format cl:nil "float64[3] angle~%uint8 mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgData)))
  "Returns full string definition for message of type 'msgData"
  (cl:format cl:nil "float64[3] angle~%uint8 mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgData>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgData>))
  "Converts a ROS message object to a list"
  (cl:list 'msgData
    (cl:cons ':angle (angle msg))
    (cl:cons ':mode (mode msg))
))
