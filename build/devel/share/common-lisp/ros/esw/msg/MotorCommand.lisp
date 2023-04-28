; Auto-generated. Do not edit!


(cl:in-package esw-msg)


;//! \htmlinclude MotorCommand.msg.html

(cl:defclass <MotorCommand> (roslisp-msg-protocol:ros-message)
  ((motor1_speed
    :reader motor1_speed
    :initarg :motor1_speed
    :type cl:float
    :initform 0.0)
   (motor2_speed
    :reader motor2_speed
    :initarg :motor2_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorCommand (<MotorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name esw-msg:<MotorCommand> is deprecated: use esw-msg:MotorCommand instead.")))

(cl:ensure-generic-function 'motor1_speed-val :lambda-list '(m))
(cl:defmethod motor1_speed-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader esw-msg:motor1_speed-val is deprecated.  Use esw-msg:motor1_speed instead.")
  (motor1_speed m))

(cl:ensure-generic-function 'motor2_speed-val :lambda-list '(m))
(cl:defmethod motor2_speed-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader esw-msg:motor2_speed-val is deprecated.  Use esw-msg:motor2_speed instead.")
  (motor2_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommand>) ostream)
  "Serializes a message object of type '<MotorCommand>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor1_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor2_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommand>) istream)
  "Deserializes a message object of type '<MotorCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor1_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor2_speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommand>)))
  "Returns string type for a message object of type '<MotorCommand>"
  "esw/MotorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommand)))
  "Returns string type for a message object of type 'MotorCommand"
  "esw/MotorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommand>)))
  "Returns md5sum for a message object of type '<MotorCommand>"
  "44cbd85de4c6dd4929217d06c0fc9617")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommand)))
  "Returns md5sum for a message object of type 'MotorCommand"
  "44cbd85de4c6dd4929217d06c0fc9617")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommand>)))
  "Returns full string definition for message of type '<MotorCommand>"
  (cl:format cl:nil "float32 motor1_speed~%float32 motor2_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommand)))
  "Returns full string definition for message of type 'MotorCommand"
  (cl:format cl:nil "float32 motor1_speed~%float32 motor2_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommand>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommand
    (cl:cons ':motor1_speed (motor1_speed msg))
    (cl:cons ':motor2_speed (motor2_speed msg))
))
