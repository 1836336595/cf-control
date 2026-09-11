;; Auto-generated. Do not edit!


(when (boundp 'crazyswarm::CTBR)
  (if (not (find-package "CRAZYSWARM"))
    (make-package "CRAZYSWARM"))
  (shadow 'CTBR (find-package "CRAZYSWARM")))
(unless (find-package "CRAZYSWARM::CTBR")
  (make-package "CRAZYSWARM::CTBR"))

(in-package "ROS")
;;//! \htmlinclude CTBR.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass crazyswarm::CTBR
  :super ros::object
  :slots (_header _body_rates _collective_thrust _thrust_raw_scale ))

(defmethod crazyswarm::CTBR
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:body_rates __body_rates) (instance geometry_msgs::Vector3 :init))
    ((:collective_thrust __collective_thrust) 0.0)
    ((:thrust_raw_scale __thrust_raw_scale) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _body_rates __body_rates)
   (setq _collective_thrust (float __collective_thrust))
   (setq _thrust_raw_scale (float __thrust_raw_scale))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:body_rates
   (&rest __body_rates)
   (if (keywordp (car __body_rates))
       (send* _body_rates __body_rates)
     (progn
       (if __body_rates (setq _body_rates (car __body_rates)))
       _body_rates)))
  (:collective_thrust
   (&optional __collective_thrust)
   (if __collective_thrust (setq _collective_thrust __collective_thrust)) _collective_thrust)
  (:thrust_raw_scale
   (&optional __thrust_raw_scale)
   (if __thrust_raw_scale (setq _thrust_raw_scale __thrust_raw_scale)) _thrust_raw_scale)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _body_rates
    (send _body_rates :serialization-length)
    ;; float32 _collective_thrust
    4
    ;; float32 _thrust_raw_scale
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _body_rates
       (send _body_rates :serialize s)
     ;; float32 _collective_thrust
       (sys::poke _collective_thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thrust_raw_scale
       (sys::poke _thrust_raw_scale (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _body_rates
     (send _body_rates :deserialize buf ptr-) (incf ptr- (send _body_rates :serialization-length))
   ;; float32 _collective_thrust
     (setq _collective_thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thrust_raw_scale
     (setq _thrust_raw_scale (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get crazyswarm::CTBR :md5sum-) "12512d5e6e983bf9418befab5e94027f")
(setf (get crazyswarm::CTBR :datatype-) "crazyswarm/CTBR")
(setf (get crazyswarm::CTBR :definition-)
      "std_msgs/Header header
geometry_msgs/Vector3 body_rates
float32 collective_thrust
# 仅作用于主机桥接层的 raw PWM 映射；0 表示兼容旧发布者并使用 1.0。
# 控制器在起飞前由冻结的电压中位数计算该值，飞行中不得逐样本改变。
float32 thrust_raw_scale

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :crazyswarm/CTBR "12512d5e6e983bf9418befab5e94027f")


