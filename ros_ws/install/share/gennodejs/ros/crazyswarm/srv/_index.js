
"use strict";

let Land = require('./Land.js')
let UploadTrajectory = require('./UploadTrajectory.js')
let UpdateParams = require('./UpdateParams.js')
let GoTo = require('./GoTo.js')
let Stop = require('./Stop.js')
let NotifySetpointsStop = require('./NotifySetpointsStop.js')
let SetGroupMask = require('./SetGroupMask.js')
let Takeoff = require('./Takeoff.js')
let StartTrajectory = require('./StartTrajectory.js')

module.exports = {
  Land: Land,
  UploadTrajectory: UploadTrajectory,
  UpdateParams: UpdateParams,
  GoTo: GoTo,
  Stop: Stop,
  NotifySetpointsStop: NotifySetpointsStop,
  SetGroupMask: SetGroupMask,
  Takeoff: Takeoff,
  StartTrajectory: StartTrajectory,
};
