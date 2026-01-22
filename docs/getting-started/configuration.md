
### Example Cosys-AirSim Settings

Example `settings.json`

``` JSON
{
    "SettingsVersion": 2.0,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlIp": "192.168.128.1",
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "LocalHostIp": "192.168.128.1",
            "Sensors":{
                "Barometer":{
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                },
				"Lidar1": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 16,
                    "PointsPerSecond": 10000,
                    "X": 0, "Y": 0, "Z": -1,
                    "DrawDebugPoints": false
                }
            },
			"Cameras": {
				"FrontCam": {
				  "CaptureSettings": [
					{
					  "ImageType": 0,
					  "Width": 640,
					  "Height": 480,
					  "FOV_Degrees": 90,
					  "AutoExposureSpeed": 100,
					  "AutoExposureBias": 0,
					  "AutoExposureMaxBrightness": 0.64,
					  "AutoExposureMinBrightness": 0.03,
					  "MotionBlurAmount": 0,
					  "TargetGamma": 1.0
					}
				  ],
				  "X": 0, "Y": 0.0, "Z": 0.20,
				  "Pitch": -10.00, "Roll": 0.0, "Yaw": 0.0
				}
			},
			"Parameters": {
				"NAV_RCL_ACT": 0,
				"NAV_DLL_ACT": 0,
				"COM_OBL_ACT": 1,
				"LPE_LAT": 47.641468,
				"LPE_LON": -122.140165
			}
        }
    }
}

```