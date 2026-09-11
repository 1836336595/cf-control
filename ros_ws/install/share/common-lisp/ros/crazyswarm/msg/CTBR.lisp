; Auto-generated. Do not edit!


(cl:in-package crazyswarm-msg)


;//! \htmlinclude CTBR.msg.html

(cl:defclass <CTBR> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (body_rates
    :reader body_rates
    :initarg :body_rates
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (collective_thrust
    :reader collective_thrust
    :initarg :collective_thrust
    :type cl:float
    :initform 0.0)
   (thrust_raw_scale
    :reader thrust_raw_scale
    :initarg :thrust_raw_scale
    :type cl:float
    :initform 0.0))
)

(cl:defclass CTBR (<CTBR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CTBR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CTBR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazyswarm-msg:<CTBR> is deprecated: use crazyswarm-msg:CTBR instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CTBR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyswarm-msg:header-val is deprecated.  Use crazyswarm-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'body_rates-val :lambda-list '(m))
(cl:defmethod body_rates-val ((m <CTBR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyswarm-msg:body_rates-val is deprecated.  Use crazyswarm-msg:body_rates instead.")
  (body_rates m))

(cl:ensure-generic-function 'collective_thrust-val :lambda-list '(m))
(cl:defmethod collective_thrust-val ((m <CTBR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyswarm-msg:collective_thrust-val is deprecated.  Use crazyswarm-msg:collective_thrust instead.")
  (collective_thrust m))

(cl:ensure-generic-function 'thrust_raw_scale-val :lambda-list '(m))
(cl:defmethod thrust_raw_scale-val ((m <CTBR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazyswarm-msg:thrust_raw_scale-val is deprecated.  Use crazyswarm-msg:thrust_raw_scale instead.")
  (thrust_raw_scale m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CTBR>) ostream)
  "Serializes a message object of type '<CTBR>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'body_rates) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'collective_thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thrust_raw_scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CTBR>) istream)
  "Deserializes a message object of type '<CTBR>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'body_rates) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'collective_thrust) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_raw_scale) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CTBR>)))
  "Returns string type for a message object of type '<CTBR>"
  "crazyswarm/CTBR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CTBR)))
  "Returns string type for a message object of type 'CTBR"
  "crazyswarm/CTBR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CTBR>)))
  "Returns md5sum for a message object of type '<CTBR>"
  "12512d5e6e983bf9418befab5e94027f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CTBR)))
  "Returns md5sum for a message object of type 'CTBR"
  "12512d5e6e983bf9418befab5e94027f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CTBR>)))
  "Returns full string definition for message of type '<CTBR>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Vector3 body_rates~%float32 collective_thrust~%# 仅作用于主机桥接层的 raw PWM 映射；0 表示兼容旧发布者并使用 1.0。~%# 控制器在起飞前由冻结的电压中位数计算该值，飞行中不得逐样本改变。~%float32 thrust_raw_scale~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CTBR)))
  "Returns full string definition for message of type 'CTBR"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Vector3 body_rates~%float32 collective_thrust~%# 仅作用于主机桥接层的 raw PWM 映射；0 表示兼容旧发布者并使用 1.0。~%# 控制器在起飞前由冻结的电压中位数计算该值，飞行中不得逐样本改变。~%float32 thrust_raw_scale~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CTBR>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'body_rates))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CTBR>))
  "Converts a ROS message object to a list"
  (cl:list 'CTBR
    (cl:cons ':header (header msg))
    (cl:cons ':body_rates (body_rates msg))
    (cl:cons ':collective_thrust (collective_thrust msg))
    (cl:cons ':thrust_raw_scale (thrust_raw_scale msg))
))
