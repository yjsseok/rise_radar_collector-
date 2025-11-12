Sensor-ext
note
This is a special resource to control sensors in SENSR. You can add, delete or modify sensors via this resource.

GET
/[SENSR version]/settings/sensor-ext/list
Get the list of available config-id and config data type of a specific type of sensor.

Parameters
sensor-type : The type of sensor(lidar/odom/can)
Return code
200 OK
404 NOT FOUND
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/sensor-ext/list?sensor-type=lidar'
** Respond **
[
    {
        "uid": "String",
        "name": "String",
        "topic": "String",
        "sensor": "String",
        "stack_capacity": "Int",
        "detection_radius": "Float",
        "horizontal_angle_range": "Vector2",
        "use_intensity_filtering": "Bool",
        "use_lidar_timestamp": "Bool",
        "use_corrected_pose": "Bool",
        "save_corrected_pose": "Bool",
        "disabled": "Bool",
        "minimum_intensity": "Float",
        "retro_reflective_intensity": "Float",
        "arguments": "Table",
        "connected_to_edge_node": "Bool",
        "edge_uid": "String",
        "base_to_origin": {
            "tx": "Float",
            "ty": "Float"
        },
        "sensor_to_base": {
            "tz": "Float",
            "qw": "Float",
            "qx": "Float",
            "qy": "Float",
            "qz": "Float"
        },
        "pose_correction": {
            "tx": "Float",
            "ty": "Float",
            "tz": "Float",
            "qw": "Float",
            "qx": "Float",
            "qy": "Float",
            "qz": "Float"
        }
    }
]


/[SENSR version]/settings/sensor-ext
Get an actual configuration settings of the desired sensor.

If you do not pass any sensor-id, this command returns the full sensor IDs in SENSR. You can filter this list by passing in node-uri to filter out only the sensors beloning to a specified algo node.

Parameters
sensor-id : ID of sensor.
node-uri (optional) : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/sensor-ext?sensor-id=lidar_0000'
** Respond **
{
    "uid": "lidar_0000",
    "name": "ROS Message",
    "topic": "/lidar_0001/pandar",
    "sensor": "ROS Message",
    "connected_to_edge_node": false,
    "edge_uid": "",
    "stack_capacity": 0,
    "detection_radius": 1000,
    "horizontal_angle_range": [
        -180,
        180
    ],
    "use_lidar_timestamp": false,
    "use_corrected_pose": false,
    "save_corrected_pose": true,
    "use_intensity_filtering": false,
    "disabled": false,
    "minimum_intensity": 0,
    "retro_reflective_intensity": 255,
    "base_to_origin": {
        "tx": -12.603933334350586,
        "ty": -6.328624725341797
    },
    "sensor_to_base": {
        "tz": 4.517395496368408,
        "qw": -0.130505308508873,
        "qx": 0.07877259701490402,
        "qy": 0.04761473089456558,
        "qz": 0.9871656894683838
    },
    "pose_correction": {
        "tx": 0,
        "ty": 0,
        "tz": 0,
        "qw": 0,
        "qx": 0,
        "qy": 0,
        "qz": 0
    },
    "arguments":{}
}

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/sensor-ext'
** Respond **
[
    "lidar_0000",
    "lidar_0001"
]

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/sensor-ext?node-uri=algo_0000'
** Respond **
[
    "lidar_0000"
]



PUT
/[SENSR version]/settings/sensor-ext
Add a new sensor to the desired algo node. If this command was successful the ID of the newly added sensor is returned.

You should pass in the sensor data with this PUT command via the request body.

You should also call /commands/apply-change to apply and save your changes. Without this the changes will not properly take effect in SENSR.

List of valid sensor

"ROS Message"
"Velodyne 16"
"Velodyne 32"
"Velodyne 64"
"Velodyne 128"
"Velodyne HDL-32"
"Hesai 20A"
"Hesai 20B"
"Hesai 40P"
"Hesai 40M"
"Hesai XT-16"
"Hesai XT-32"
"Hesai QT"
"Hesai 64"
"Livox"
"Innoviz Pro" // X86_64 only
"Innoviz One" // X86_64 only
"Ouster-OS"
"Baraja"
"Cepton Vista"
"Quanergy"
"Quanergy MQ-8"
"Blickfeld Cube 1"
"Blickfeld Cube Range 1"
"Luminar Hydra"
"Innovusion Falcon"
"AEye 4Sight M"
Parameters
node-uri : ID of the desired algo node.
sensor-type : The type of sensor (lidar/odom/can)
Return code
200 OK
400 BAD REQ
Example

$ curl -X PUT 'http://localhost:9080/[SENSR version]/settings/sensor-ext?node-uri=algo_0000&sensor-type=lidar'
--data '{
    "name": "PointCloud",
    "topic": "/velodyne_points",
    "sensor": "ROS Message",
    "stack_capacity": 0,
    "detection_radius": 1000.0,
    "horizontal_angle_range": [-180.0, 180.0],
    "use_lidar_timestamp" : false,
    "use_intensity_filtering" : false,
    "use_corrected_pose":false,
    "save_corrected_pose":false,
    "minimum_intensity": 0.0,
    "retro_reflective_intensity": 200.0,
    "connected_to_edge_node": false,
    "edge_uid":"",
    "base_to_origin": {
        "tx": 0.0,
        "ty": 0.0
    },
    "sensor_to_base": {
        "tz": 0.0,
        "qw": 1.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0
    },
    "pose_correction": {
        "tx": 0.0,
        "ty": 0.0,
        "tz": 0.0,
        "qw": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0
    },
    "disabled": false,
    "arguments": {}
}'
** Respond **
"lidar_0001"



POST
/[SENSR version]/settings/sensor-ext
Update the config of the desired sensor. Note that topic should be unique within an algo node.

See the PUT section for the list of valid sensors.

You should also call /commands/apply-change to apply your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example
$ curl -X POST 'http://localhost:9080/[SENSR version]/settings/sensor-ext' 
--data '{
    "name": "PointCloud",
    "topic": "/velodyne_points",
    "sensor": "ROS Message",
    "stack_capacity": 0,
    "detection_radius": 1000.0,
    "horizontal_angle_range": [-180.0, 180.0],
    "use_lidar_timestamp" : false,
    "use_intensity_filtering" : false,
    "use_corrected_pose":false,
    "save_corrected_pose":false,
    "minimum_intensity": 0.0,
    "retro_reflective_intensity": 200.0,
    "connected_to_edge_node": false,
    "edge_uid":"",
    "base_to_origin": {
        "tx": 0.0,
        "ty": 0.0
    },
    "sensor_to_base": {
        "tz": 0.0,
        "qw": 1.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0
    },
    "pose_correction": {
        "tx": 0.0,
        "ty": 0.0,
        "tz": 0.0,
        "qw": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
    },
    "disabled": false,
    "arguments": {}
}'
** Respond **
200 OK



DELETE
/[SENSR version]/settings/sensor-ext
Delete a sensor.

Parameters
sensor-id : ID of sensor.
Return code
200 OK
400 BAD REQ
Example

$ curl -X DELETE 'http://localhost:9080/[SENSR version]/settings/sensor-ext?sensor-id=lidar_0001'        
** Respond **
200 OK

Node
note
This resource is used for interacting with algo node configurations in SENSR. A node configuration has configs related to algo node setup. (e.g. username, bin ...)

GET
/[SENSR version]/settings/node/list
Get the list of available config IDs and associated data types of an algo node.

Parameters
N/A
Return code
200 OK
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/node/list'
** Respond **
[
    {
        "uid": "String",
        "ip": "String",
        "name": "String",
        "rosbag_path": "String",
        "preset": "String",
        "enable_gpu": "Bool",
        "disabled": "Bool"
    }
]


/[SENSR version]/settings/node
Get an config table of the desired algo node.

Parameters
node-uri : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/node?node-uri=algo_0000'
** Respond **
{
    "uid":"algo_0000",
    "ip":"172.17.0.1",
    "name":"Algo Node 1",
    "rosbag_path":"/root/seoulrobotics/rosbags/sample.bag",
    "preset":"outdoor_gpu",
    "enable_gpu":true,
    "disabled":false
}


/[SENSR version]/settings/node-transform
Get the transform (position and rotation) of the algo node

Parameters
node-uri : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/node-transform?node-uri=algo_0000'
** Respond **
{
    "base_to_origin": {
        "tx": 0.0,
        "ty": 0.0
    },
    "sensor_to_base": {
        "tz": 0.0,
        "qw": 1.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0
    }
}


/[SENSR version]/settings/node-detection-range
Get the detection range of the algo node

Parameters
node-uri : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/node-detection-range?node-uri=algo_0000'
** Respond **
{
    "x_range": [
        -50.0,
        50.0
    ],
    "y_range": [
        -10.0,
        50.0
    ],
    "z_range": [
        -1.5,
        3.5
    ]
}



PUT
/[SENSR version]/settings/node
This command creates a new algo node.

The command takes in the algo node configuration table as can be seen in the example below. Please refer to the POST command section for details about the format of the data as it is almost identical. The only difference is that you can skip the uid here, SENSR will create one for the newly created algo node.

If this command was successful, the uid of the new algo node is returned.

You should also call /commands/apply-change to save your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X PUT 'http://localhost:9080/[SENSR version]/settings/node'
--data '{
    "ip": "192.168.0.100",
    "name": "Algo_1",
    "rosbag_path": "",
    "preset": "outdoor",
    "enable_gpu": false,
    "disabled":false
}'
** Respond **
"algo_0001"



POST
/[SENSR version]/settings/node
Update the config of the algo node specified by the uid-field in the request body. Note that the combination of IP address and port should be unique.

You should also call /commands/apply-change to save your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/settings/node'
--data '{
    "uid": "algo_0000",
    "ip": "192.168.0.100",
    "name": "Algo_1",
    "rosbag_path": "",
    "preset": "outdoor",
    "enable_gpu": false,
    "disabled":false
}'
** Respond **
200 OK


/[SENSR version]/settings/node-transform
Update the transform (position and rotation) of the algo node specified by the uid-parameter.

The fields "qw", "qx", "qy", and "qz" are the components of the quaternion rotation. The rotation of the algo node is also limited to the z-axis, so "qx" and "qy" should be kept as zero. Additionally the norm of the quaternion should be 1.0 for it to be considered valid.

Note that for the algo node the height should always be ground-level, so keep "tz" as zero.

You should also call /commands/apply-change to save your changes.

Parameters
node-uri : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/settings/node-transform?node-uri=algo_0000'
--data '{
    "base_to_origin": {
        "tx": 1.0,
        "ty": 2.0
    },
    "sensor_to_base": {
        "tz": 0.0,
        "qw": 1.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0
    }
}'
** Respond **
200 OK


/[SENSR version]/settings/node-detection-range
Update the detection range of the algo node specified by the uid-parameter.

You should also call /commands/apply-change to save your changes.

Parameters
node-uri : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/settings/node-detection-range?node-uri=algo_0000' 
--data '{
    "x_range": [
        -20.0,
        10.0
    ],
    "y_range": [
        -10.0,
        50.0
    ],
    "z_range": [
        -1.5,
        3.5
    ]
}'
** Respond **
200 OK



DELETE
/[SENSR version]/settings/node
Delete an algo node.

Parameters
node-uri : ID of algo node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X DELETE 'http://localhost:9080/[SENSR version]/settings/node?node-uri=algo_0001'        
** Respond **
200 OK

Edge Node
note
This resource is used for interacting with edge node configurations in SENSR. A node configuration has configs related to edge node setup. (e.g. username, bin ...)

GET
/[SENSR version]/settings/edge-node
Get an config table of the desired edge node.

Parameters
edge-node-uri : ID of edge node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/edge-node?edge-node-uri=edge_0000'
** Respond **
{
    "uid":"edge_0000",
    "ip":"172.17.0.1",
    "name":"Edge Node 1",
    "disabled":false
}



PUT
/[SENSR version]/settings/edge-node
This command creates a new edge node.

The command takes in the edge node configuration table as can be seen in the example below. Please refer to the POST command section for details about the format of the data as it is almost identical. The only difference is that you can skip the uid here, SENSR will create one for the newly created edge node.

If this command was successful, the uid of the new edge node is returned.

You should also call /commands/apply-change to save your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X PUT 'http://localhost:9080/[SENSR version]/settings/edge-node'
--data '{
    "algo_uid": "algo_0000",
    "ip": "192.168.0.100",
    "name": "Edge_1",
    "disabled":false
}'
** Respond **
"edge_0001"



POST
/[SENSR version]/settings/edge-node
Update the config of the edge node specified by the uid-field in the request body. Note that the combination of IP address and port should be unique.

You should also call /commands/apply-change to save your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/settings/edge-node'
--data '{
    "uid": "edge_0001",
    "ip": "192.168.0.100",
    "name": "Edge_1 Updated",
    "disabled":false
}'
** Respond **
200 OK



DELETE
/[SENSR version]/settings/edge-node
Delete an edge node.

Parameters
edge-node-uri : ID of edge node.
Return code
200 OK
400 BAD REQ
Example

$ curl -X DELETE 'http://localhost:9080/[SENSR version]/settings/edge-node?edge-node-uri=edge_0001'        
** Respond **
200 OK

Zone
note
This resource shows how to interact with zone configurations in SENSR. You can add, delete or modify zones via this interface.

GET
/[SENSR version]/settings/zone/list
Get the list of available config-id and config data type of a zone.

Parameters
N/A
Return code
200 OK
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/zone/list'
** Respond **
[
    {
        "id": "Int",
        "name": "String",
        "zone_type": "String",
        "speed_limit": "Float",
        "loitering_threshold": "Int",
        "min_z": "Float",
        "max_z": "Float",
        "vertices": "Array[Vector2]"
    }
]


/[SENSR version]/settings/zone
Get an actual config of the desired zone

Parameters
zone-id : ID of zone. If you do not pass any zone-id this command returns the full list of zone IDs in the project.
Return code
200 OK
400 BAD REQ
Example

curl -X GET 'http://localhost:9080/[SENSR version]/settings/zone'
** Respond **
[
    1001,
    1002
]

$ curl -X GET 'http://localhost:9080/[SENSR version]/settings/zone?zone-id=1001'
** Respond **
{
    "id": 1001,
    "name": "zone-1001",
    "zone_type": "Event",
    "speed_limit": 27.77,
    "loitering_threshold": 0,
    "min_z": 0.0,
    "max_z": 2.5,
    "vertices": [
        [
            8.95,
            -21.91
        ],
        [
            -13.30,
            -25.27
        ],
        [
            -12.95,
            -4.06
        ],
        [
            16.37,
            5.74
        ],
        [
            9.58,
            -21.35
        ]
    ]
}



PUT
/[SENSR version]/settings/zone
Creates a new zone and adds it to the project. If this command was successful, the ID of newly added zone is returned.

The id-field in the body will be ignored, SENSR will assign a new ID to the newly created zone.

You should also call /commands/apply-change to apply your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X PUT 'http://localhost:9080/[SENSR version]/settings/zone' 
--data '{
    "id": 1001,
    "name": "zone-1001",
    "zone_type": "Event",
    "speed_limit": 27.77,
    "loitering_threshold": 0,
    "min_z": 0.0,
    "max_z": 2.5,
    "vertices": [
        [
            -4.04,
            5.27
        ],
        [
            -3.68,
            3.32
        ],
        [
            -5.65,
            3.02
        ],
        [
            -5.96,
            4.85
        ]
    ]
}'
** Respond **
"1001



POST
/[SENSR version]/settings/zone
Update the configuration of an existing zone. The id-field should correspond to the ID of the zone to be updated.

You should also call /commands/apply-change to apply your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/settings/zone' 
--data '{
    "id": 1001,
    "name": "zone-1001",
    "zone_type": "Event",
    "speed_limit": 27.77,
    "loitering_threshold": 0,
    "min_z": 0.0,
    "max_z": 2.5,
    "vertices": [
        [
            -4.04,
            5.27
        ],
        [
            -3.68,
            3.32
        ],
        [
            -5.65,
            3.02
        ],
        [
            -5.96,
            4.85
        ]
    ]
}'
** Respond **
200 OK



DELETE
/[SENSR version]/settings/zone
Delete a specific zone.

You should also call /commands/apply-change to apply your changes.

Parameters
zone-id : ID of zone to be deleted.
Return code
200 OK
400 BAD REQ
Example

$ curl -X DELETE 'http://localhost:9080/[SENSR version]/settings/zone?zone-id=1001'        
** Respond **
200 OK


/[SENSR version]/settings/zone-all
Delete all zones in the project.

You should also call /commands/apply-change to apply your changes.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X DELETE 'http://localhost:9080/[SENSR version]/settings/zone-all'        
** Respond **
200 OK

Results
note
This resource demonstrates how to query detailed information of an object, zone or other health. It provides Object Shape, Object History, Zone Configuration and SENSR's Health Status.

We recommend using the SENSR SDK over this when possible as it provides a more stable interface.

GET
/results/object/shape
Return point clouds of an object at a specific timestamp. If a timestamp is not provided the current time will be used.

If the timestamp does not match with any timestamp SENSR has in memory it will match with the timestamp closest to the provided one.

Parameters
id : Object ID
time : (Optional) Approx. timestamp when the event occurred. Format (e.g. 2020-12-04T06:40.125Z)
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/results/object/shape?id=0&time=2020-12-04T06:40.125000000Z'
** Respond **
{ "id" : 0,
  "timestamp" : "2020-12-04T06:40.125000000Z",
  "points" : ".............." }


/results/object/history
Return historical data of an object such as position. If length is not provided SENSR will return all available history for that object.

Parameters
id : Object ID
length : (Optional) History length
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/results/object/history?id=0&length=-1'
** Respond **
{
  "id" : 0,
  "states" : [
    {
      "position": {"x":0.0,"y":0.0,"z":0.0},
      "timestamp" : "2020-12-04T06:40.125000000Z"
    },
    ...
  ]
}


/results/zone
Return the detailed status of a specific event zone. The response has the list of object IDs that are inside the zone at annotated timing.

Note that this is only available for event zones. Other zones types will not work with this interface.

Parameters
id : Zone ID (Returns all zones if zero)
Return code
200 OK
Example

$ curl -X GET 'http://localhost:9080/results/zone?id=1001'
** Respond **
{
  "zones":[{
    "id":1001,
    "name":"Default Name",
    "pbox":{
      "points":[
        {"x":6.75325871,"y":-32.13},
        {"x":-18.3767433,"y":-24.1500034},
        {"x":-11.7967463,"y":20.0199947},
        {"x":12.7032528,"y":3.29000163}
      ],
      "minZ":-1.5,
      "maxZ":3.5
      },
      "type":"event",
      "objectIds":[43,26,20,5]
    }]
  }


/results/health
Return the current system health.

Parameters
Return code
200 OK
Example

$ curl -X GET 'http://localhost:9080/results/health'
** Respond **
{
  "master":"OK",
  "nodes":{
    "algo_0000":{
      "status":"OK",
      "sensors":{
        "/lidar_0003/pandar":"SENSOR_ALIVE",
        "/lidar_0002/pandar":"SENSOR_ALIVE",
        "/lidar_0001/pandar":"SENSOR_ALIVE",
        "/lidar_0000/pandar":"SENSOR_ALIVE"
      }
    }
  }
}

Commands
note
This resource allows user to request a special behaviour to SENSR. Detailed behaviour of each commands are described below.

POST
/[SENSR version]/commands/start
Request to start or transition to a new mode.

Parameters
mode : The new mode. This can be either calibration (project setup mode) or runtime.
Return code
200 OK
400 BAD REQ
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/start?mode=calibration'
$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/start?mode=runtime'


/[SENSR version]/commands/apply-change
Save the project and apply all the configuration settings that have been changed (algo nodes, sensors, zones, algorithm parameters, etc).

Parameters
N/A
Return code
200 OK
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/apply-change'


/[SENSR version]/commands/reload
Reload the current project from saved configuration. This will revert any unsaved changes to the settings.

Parameters
N/A
Return code
200 OK
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/reload'


/[SENSR version]/commands/cancel
Request to cancel mode switching. This command is useful when SENSR does not respond because of wrong setup of algo node.

Parameters
N/A
Return code
200 OK
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/cancel'

Project
note
This resource provides convenient interfaces to interact with SENSR projects, such as creating or loading existing projects. Detailed behavior of each command is described below.

GET
/[SENSR version]/commands/project/list
Return all available projects in path.

Parameters
path : Path for where to search for SENSR projects. If not specified then SENSR will use the default project path /root/seoulrobotics/projects which is mounted to /opt/seoulrobotics/projects in the local machine. This needs to be an absolute path.
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/commands/project/list'
** Respond **
[
    "project_1",
    "project_2",
    "project_3"
]

$ curl -X GET 'http://localhost:9080/[SENSR version]/commands/project/list?path=/root/seoulrobotics/projects'
** Respond **
No projects found under "/root/seoulrobotics/projects".


/[SENSR version]/commands/project
Return information about the currently open project.

Parameters
N/A
Return code
200 OK
400 BAD REQ
Example

$ curl -X GET 'http://localhost:9080/[SENSR version]/commands/project'
** Respond **
{
    "project_name": "my_project"
}



PUT
/[SENSR version]/commands/project
Create a new project. If another SENSR project is open it will close and

Parameters
path : Path where the project will be created. If not specified then the default path /root/seoulrobotics/projects will be used.
name : The name of the new project.
Return code
200 OK
400 BAD REQ
500 INTERNAL SERVER ERR
Example

$ curl -X PUT 'http://localhost:9080/[SENSR version]/commands/project?name=my-new-project'
** Respond **
Successfully created project "my-new-project".



POST
/[SENSR version]/commands/project
Load an existing project. If the project does not exist SENSR will return an error.

Parameters
path : Parent path of the project located. If not specified then the default path /root/seoulrobotics/projects will be used.
name : Name of the project.
Return code
200 OK
400 BAD REQ
500 INTERNAL SERVER ERR
Example

$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/project?name=my-project'
** Respond **
Successfully opened "my-project".

$ curl -X POST 'http://localhost:9080/[SENSR version]/commands/project?path=/root/seoulrobotics/projects&name=project-that-does-not-exist'
** Respond **
Failed to open "project-that-does-not-exist".