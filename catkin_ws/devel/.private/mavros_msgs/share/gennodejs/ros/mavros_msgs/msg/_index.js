
"use strict";

let GPSINPUT = require('./GPSINPUT.js');
let RCIn = require('./RCIn.js');
let ParamValue = require('./ParamValue.js');
let SysStatus = require('./SysStatus.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let GPSRTK = require('./GPSRTK.js');
let Thrust = require('./Thrust.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let TerrainReport = require('./TerrainReport.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let DebugValue = require('./DebugValue.js');
let VehicleInfo = require('./VehicleInfo.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let Trajectory = require('./Trajectory.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let ESCStatus = require('./ESCStatus.js');
let FileEntry = require('./FileEntry.js');
let PositionTarget = require('./PositionTarget.js');
let Param = require('./Param.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let BatteryStatus = require('./BatteryStatus.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let Mavlink = require('./Mavlink.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let Vibration = require('./Vibration.js');
let Altitude = require('./Altitude.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let LogData = require('./LogData.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let CommandCode = require('./CommandCode.js');
let HilSensor = require('./HilSensor.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let RadioStatus = require('./RadioStatus.js');
let RTCM = require('./RTCM.js');
let LandingTarget = require('./LandingTarget.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let LogEntry = require('./LogEntry.js');
let HilControls = require('./HilControls.js');
let RTKBaseline = require('./RTKBaseline.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let HomePosition = require('./HomePosition.js');
let Tunnel = require('./Tunnel.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let Waypoint = require('./Waypoint.js');
let ManualControl = require('./ManualControl.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let WaypointReached = require('./WaypointReached.js');
let ExtendedState = require('./ExtendedState.js');
let ActuatorControl = require('./ActuatorControl.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let HilGPS = require('./HilGPS.js');
let MountControl = require('./MountControl.js');
let ESCInfo = require('./ESCInfo.js');
let StatusText = require('./StatusText.js');
let CellularStatus = require('./CellularStatus.js');
let WaypointList = require('./WaypointList.js');
let VFR_HUD = require('./VFR_HUD.js');
let State = require('./State.js');
let GPSRAW = require('./GPSRAW.js');
let RCOut = require('./RCOut.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');

module.exports = {
  GPSINPUT: GPSINPUT,
  RCIn: RCIn,
  ParamValue: ParamValue,
  SysStatus: SysStatus,
  CameraImageCaptured: CameraImageCaptured,
  GPSRTK: GPSRTK,
  Thrust: Thrust,
  OpticalFlowRad: OpticalFlowRad,
  TerrainReport: TerrainReport,
  WheelOdomStamped: WheelOdomStamped,
  DebugValue: DebugValue,
  VehicleInfo: VehicleInfo,
  PlayTuneV2: PlayTuneV2,
  Trajectory: Trajectory,
  ESCStatusItem: ESCStatusItem,
  ESCStatus: ESCStatus,
  FileEntry: FileEntry,
  PositionTarget: PositionTarget,
  Param: Param,
  ESCInfoItem: ESCInfoItem,
  BatteryStatus: BatteryStatus,
  ADSBVehicle: ADSBVehicle,
  Mavlink: Mavlink,
  OverrideRCIn: OverrideRCIn,
  Vibration: Vibration,
  Altitude: Altitude,
  CompanionProcessStatus: CompanionProcessStatus,
  HilStateQuaternion: HilStateQuaternion,
  LogData: LogData,
  GlobalPositionTarget: GlobalPositionTarget,
  CommandCode: CommandCode,
  HilSensor: HilSensor,
  CamIMUStamp: CamIMUStamp,
  RadioStatus: RadioStatus,
  RTCM: RTCM,
  LandingTarget: LandingTarget,
  TimesyncStatus: TimesyncStatus,
  LogEntry: LogEntry,
  HilControls: HilControls,
  RTKBaseline: RTKBaseline,
  MagnetometerReporter: MagnetometerReporter,
  ESCTelemetry: ESCTelemetry,
  HomePosition: HomePosition,
  Tunnel: Tunnel,
  ESCTelemetryItem: ESCTelemetryItem,
  Waypoint: Waypoint,
  ManualControl: ManualControl,
  HilActuatorControls: HilActuatorControls,
  WaypointReached: WaypointReached,
  ExtendedState: ExtendedState,
  ActuatorControl: ActuatorControl,
  AttitudeTarget: AttitudeTarget,
  NavControllerOutput: NavControllerOutput,
  EstimatorStatus: EstimatorStatus,
  HilGPS: HilGPS,
  MountControl: MountControl,
  ESCInfo: ESCInfo,
  StatusText: StatusText,
  CellularStatus: CellularStatus,
  WaypointList: WaypointList,
  VFR_HUD: VFR_HUD,
  State: State,
  GPSRAW: GPSRAW,
  RCOut: RCOut,
  OnboardComputerStatus: OnboardComputerStatus,
};
