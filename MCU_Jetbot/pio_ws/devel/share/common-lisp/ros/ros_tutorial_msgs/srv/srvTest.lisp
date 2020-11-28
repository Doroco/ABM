; Auto-generated. Do not edit!


(cl:in-package ros_tutorial_msgs-srv)


;//! \htmlinclude srvTest-request.msg.html

(cl:defclass <srvTest-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass srvTest-request (<srvTest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srvTest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srvTest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_tutorial_msgs-srv:<srvTest-request> is deprecated: use ros_tutorial_msgs-srv:srvTest-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <srvTest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial_msgs-srv:a-val is deprecated.  Use ros_tutorial_msgs-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <srvTest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial_msgs-srv:b-val is deprecated.  Use ros_tutorial_msgs-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srvTest-request>) ostream)
  "Serializes a message object of type '<srvTest-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srvTest-request>) istream)
  "Deserializes a message object of type '<srvTest-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srvTest-request>)))
  "Returns string type for a service object of type '<srvTest-request>"
  "ros_tutorial_msgs/srvTestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvTest-request)))
  "Returns string type for a service object of type 'srvTest-request"
  "ros_tutorial_msgs/srvTestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srvTest-request>)))
  "Returns md5sum for a message object of type '<srvTest-request>"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srvTest-request)))
  "Returns md5sum for a message object of type 'srvTest-request"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srvTest-request>)))
  "Returns full string definition for message of type '<srvTest-request>"
  (cl:format cl:nil "int32 a~%int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srvTest-request)))
  "Returns full string definition for message of type 'srvTest-request"
  (cl:format cl:nil "int32 a~%int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srvTest-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srvTest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srvTest-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude srvTest-response.msg.html

(cl:defclass <srvTest-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass srvTest-response (<srvTest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srvTest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srvTest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_tutorial_msgs-srv:<srvTest-response> is deprecated: use ros_tutorial_msgs-srv:srvTest-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <srvTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial_msgs-srv:result-val is deprecated.  Use ros_tutorial_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srvTest-response>) ostream)
  "Serializes a message object of type '<srvTest-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srvTest-response>) istream)
  "Deserializes a message object of type '<srvTest-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srvTest-response>)))
  "Returns string type for a service object of type '<srvTest-response>"
  "ros_tutorial_msgs/srvTestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvTest-response)))
  "Returns string type for a service object of type 'srvTest-response"
  "ros_tutorial_msgs/srvTestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srvTest-response>)))
  "Returns md5sum for a message object of type '<srvTest-response>"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srvTest-response)))
  "Returns md5sum for a message object of type 'srvTest-response"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srvTest-response>)))
  "Returns full string definition for message of type '<srvTest-response>"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srvTest-response)))
  "Returns full string definition for message of type 'srvTest-response"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srvTest-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srvTest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srvTest-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srvTest)))
  'srvTest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srvTest)))
  'srvTest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvTest)))
  "Returns string type for a service object of type '<srvTest>"
  "ros_tutorial_msgs/srvTest")