
"use strict";

let FileTruncate = require('./FileTruncate.js')
let MountConfigure = require('./MountConfigure.js')
let LogRequestList = require('./LogRequestList.js')
let FileOpen = require('./FileOpen.js')
let FileRead = require('./FileRead.js')
let ParamPush = require('./ParamPush.js')
let ParamSet = require('./ParamSet.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let WaypointClear = require('./WaypointClear.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileChecksum = require('./FileChecksum.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileClose = require('./FileClose.js')
let FileWrite = require('./FileWrite.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandLong = require('./CommandLong.js')
let WaypointPull = require('./WaypointPull.js')
let FileRemove = require('./FileRemove.js')
let ParamGet = require('./ParamGet.js')
let CommandInt = require('./CommandInt.js')
let MessageInterval = require('./MessageInterval.js')
let ParamPull = require('./ParamPull.js')
let FileRename = require('./FileRename.js')
let StreamRate = require('./StreamRate.js')
let LogRequestData = require('./LogRequestData.js')
let FileList = require('./FileList.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let SetMode = require('./SetMode.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandHome = require('./CommandHome.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let WaypointPush = require('./WaypointPush.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandBool = require('./CommandBool.js')
let CommandTOL = require('./CommandTOL.js')
let CommandAck = require('./CommandAck.js')

module.exports = {
  FileTruncate: FileTruncate,
  MountConfigure: MountConfigure,
  LogRequestList: LogRequestList,
  FileOpen: FileOpen,
  FileRead: FileRead,
  ParamPush: ParamPush,
  ParamSet: ParamSet,
  LogRequestEnd: LogRequestEnd,
  WaypointClear: WaypointClear,
  WaypointSetCurrent: WaypointSetCurrent,
  FileChecksum: FileChecksum,
  CommandTriggerControl: CommandTriggerControl,
  FileClose: FileClose,
  FileWrite: FileWrite,
  SetMavFrame: SetMavFrame,
  CommandLong: CommandLong,
  WaypointPull: WaypointPull,
  FileRemove: FileRemove,
  ParamGet: ParamGet,
  CommandInt: CommandInt,
  MessageInterval: MessageInterval,
  ParamPull: ParamPull,
  FileRename: FileRename,
  StreamRate: StreamRate,
  LogRequestData: LogRequestData,
  FileList: FileList,
  VehicleInfoGet: VehicleInfoGet,
  CommandTriggerInterval: CommandTriggerInterval,
  SetMode: SetMode,
  FileRemoveDir: FileRemoveDir,
  CommandHome: CommandHome,
  CommandVtolTransition: CommandVtolTransition,
  WaypointPush: WaypointPush,
  FileMakeDir: FileMakeDir,
  CommandBool: CommandBool,
  CommandTOL: CommandTOL,
  CommandAck: CommandAck,
};
