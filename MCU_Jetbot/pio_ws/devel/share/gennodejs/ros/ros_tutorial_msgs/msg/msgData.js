// Auto-generated. Do not edit!

// (in-package ros_tutorial_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class msgData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle = null;
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgData
    // Check that the constant length array field [angle] has the right length
    if (obj.angle.length !== 3) {
      throw new Error('Unable to serialize array field angle - length must be 3')
    }
    // Serialize message field [angle]
    bufferOffset = _arraySerializer.float64(obj.angle, buffer, bufferOffset, 3);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgData
    let len;
    let data = new msgData(null);
    // Deserialize message field [angle]
    data.angle = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_tutorial_msgs/msgData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1b7cf6a5d1b828728f05b43a5c9cccb2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[3] angle
    uint8 mode
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgData(null);
    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = new Array(3).fill(0)
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

module.exports = msgData;
