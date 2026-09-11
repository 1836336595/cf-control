;; Auto-generated. Do not edit!


(when (boundp 'crazyswarm::MocapState)
  (if (not (find-package "CRAZYSWARM"))
    (make-package "CRAZYSWARM"))
  (shadow 'MocapState (find-package "CRAZYSWARM")))
(unless (find-package "CRAZYSWARM::MOCAPSTATE")
  (make-package "CRAZYSWARM::MOCAPSTATE"))

(in-package "ROS")
;;//! \htmlinclude MocapState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass crazyswarm::MocapState
  :super ros::object
  :slots (_header _pose _twist _acceleration _valid _derivatives_valid ))

(defmethod crazyswarm::MocapState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    ((:valid __valid) nil)
    ((:derivatives_valid __derivatives_valid) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _acceleration __acceleration)
   (setq _valid __valid)
   (setq _derivatives_valid __derivatives_valid)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:derivatives_valid
   (&optional (__derivatives_valid :null))
   (if (not (eq __derivatives_valid :null)) (setq _derivatives_valid __derivatives_valid)) _derivatives_valid)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ;; bool _valid
    1
    ;; bool _derivatives_valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _derivatives_valid
       (if _derivatives_valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _derivatives_valid
     (setq _derivatives_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get crazyswarm::MocapState :md5sum-) "377e9e0f6dc54563913d8188c7a150e9")
(setf (get crazyswarm::MocapState :datatype-) "crazyswarm/MocapState")
(setf (get crazyswarm::MocapState :definition-)
      "std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 acceleration
bool valid
bool derivatives_valid

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

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



(provide :crazyswarm/MocapState "377e9e0f6dc54563913d8188c7a150e9")


