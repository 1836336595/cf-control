// Auto-generated. Do not edit!

// (in-package crazyswarm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CTBR {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.body_rates = null;
      this.collective_thrust = null;
      this.thrust_raw_scale = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('body_rates')) {
        this.body_rates = initObj.body_rates
      }
      else {
        this.body_rates = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('collective_thrust')) {
        this.collective_thrust = initObj.collective_thrust
      }
      else {
        this.collective_thrust = 0.0;
      }
      if (initObj.hasOwnProperty('thrust_raw_scale')) {
        this.thrust_raw_scale = initObj.thrust_raw_scale
      }
      else {
        this.thrust_raw_scale = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CTBR
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [body_rates]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.body_rates, buffer, bufferOffset);
    // Serialize message field [collective_thrust]
    bufferOffset = _serializer.float32(obj.collective_thrust, buffer, bufferOffset);
    // Serialize message field [thrust_raw_scale]
    bufferOffset = _serializer.float32(obj.thrust_raw_scale, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CTBR
    let len;
    let data = new CTBR(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_rates]
    data.body_rates = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [collective_thrust]
    data.collective_thrust = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thrust_raw_scale]
    data.thrust_raw_scale = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'crazyswarm/CTBR';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '12512d5e6e983bf9418befab5e94027f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CTBR(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.body_rates !== undefined) {
      resolved.body_rates = geometry_msgs.msg.Vector3.Resolve(msg.body_rates)
    }
    else {
      resolved.body_rates = new geometry_msgs.msg.Vector3()
    }

    if (msg.collective_thrust !== undefined) {
      resolved.collective_thrust = msg.collective_thrust;
    }
    else {
      resolved.collective_thrust = 0.0
    }

    if (msg.thrust_raw_scale !== undefined) {
      resolved.thrust_raw_scale = msg.thrust_raw_scale;
    }
    else {
      resolved.thrust_raw_scale = 0.0
    }

    return resolved;
    }
};

module.exports = CTBR;
