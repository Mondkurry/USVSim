```log

docker run -it --rm --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="${XAUTHORITY}:/root/.Xauthority:rw" --name="heron-sim-basic" --privileged docker_basic 
... logging to /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/roslaunch-docker-desktop-1.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://docker-desktop:33755/

SUMMARY
========

PARAMETERS
 * /cmd_drive_to_thrusters/namespace: 
 * /controller/fwd_vel/imax: 25.0     
 * /controller/fwd_vel/imin: -25.0    
 * /controller/fwd_vel/kd: 20
 * /controller/fwd_vel/kf: 20
 * /controller/fwd_vel/ki: 2
 * /controller/fwd_vel/kp: 40
 * /controller/max/bck_vel: 0.5
 * /controller/max/fwd_vel: 3.0
 * /controller/twist_cmd/timeout: 1.5
 * /controller/yaw/imax: 0.0
 * /controller/yaw/imin: 0.0
 * /controller/yaw/kd: 0
 * /controller/yaw/kf: 0
 * /controller/yaw/ki: 0
 * /controller/yaw/kp: 0.3
 * /controller/yaw_rate/imax: 20.0
 * /controller/yaw_rate/imin: -20.0
 * /controller/yaw_rate/kd: 20
 * /controller/yaw_rate/kf: 10
 * /controller/yaw_rate/ki: 0.0
 * /controller/yaw_rate/kp: 60
 * /ekf_localization_node/base_link_frame: base_link
 * /ekf_localization_node/frequency: 20
 * /ekf_localization_node/imu0: imu/data
 * /ekf_localization_node/imu0_config: [False, False, Fa...
 * /ekf_localization_node/imu0_differential: False
 * /ekf_localization_node/imu0_nodelay: True
 * /ekf_localization_node/imu0_queue_size: 10
 * /ekf_localization_node/imu0_relative: False
 * /ekf_localization_node/imu0_remove_gravitational_acceleration: True
 * /ekf_localization_node/map_frame: map
 * /ekf_localization_node/odom0: odometry/gps
 * /ekf_localization_node/odom0_config: [True, True, Fals...
 * /ekf_localization_node/odom0_differential: False
 * /ekf_localization_node/odom0_nodelay: True
 * /ekf_localization_node/odom0_queue_size: 10
 * /ekf_localization_node/odom0_relative: False
 * /ekf_localization_node/odom_frame: odom
 * /ekf_localization_node/process_noise_covariance: [0.1, 0, 0, 0, 0,...
 * /ekf_localization_node/sensor_timeout: 0.1
 * /ekf_localization_node/transform_time_offset: 0.0
 * /ekf_localization_node/transform_timeout: 0.0
 * /ekf_localization_node/twist0: navsat/vel_cov
 * /ekf_localization_node/twist0_config: [False, False, Fa...
 * /ekf_localization_node/twist0_differential: False
 * /ekf_localization_node/twist0_nodelay: True
 * /ekf_localization_node/twist0_queue_size: 10
 * /ekf_localization_node/twist0_relative: False
 * /ekf_localization_node/two_d_mode: True
 * /ekf_localization_node/use_control: False
 * /ekf_localization_node/world_frame: odom
 * /gazebo/enable_ros_network: True
 * /imu_filter_madgwick/publish_debug_topics: True
 * /imu_filter_madgwick/publish_tf: False
 * /imu_filter_madgwick/use_mag: True
 * /imu_filter_madgwick/use_magnetic_field_msg: True
 * /imu_filter_madgwick/world_frame: enu
 * /navsat_transform_node/broadcast_utm_transform: True
 * /navsat_transform_node/broadcast_utm_transform_as_parent_frame: True
 * /navsat_transform_node/datum: [43.4720948, -80....
 * /navsat_transform_node/delay: 0.0
 * /navsat_transform_node/frequency: 20
 * /navsat_transform_node/magnetic_declination_radians: 0
 * /navsat_transform_node/publish_filtered_gps: False
 * /navsat_transform_node/use_odometry_yaw: True
 * /navsat_transform_node/wait_for_datum: False
 * /navsat_transform_node/yaw_offset: 0.0
 * /navsat_transform_node/zero_altitude: False
 * /publish_world_models/meshes/east/plane: [2000, 0.1, 100]
 * /publish_world_models/meshes/east/pose/position: [0, 1000, -50]
 * /publish_world_models/meshes/heightmap/mesh: package://uuv_gaz...
 * /publish_world_models/meshes/heightmap/model: sand_heightmap
 * /publish_world_models/meshes/north/plane: [0.1, 2000, 100]
 * /publish_world_models/meshes/north/pose/position: [1000, 0, -50]
 * /publish_world_models/meshes/seafloor/plane: [2000, 2000, 0.1]
 * /publish_world_models/meshes/seafloor/pose/position: [0, 0, -100]
 * /publish_world_models/meshes/south/plane: [0.1, 2000, 100]
 * /publish_world_models/meshes/south/pose/position: [-1000, 0, -50]
 * /publish_world_models/meshes/west/plane: [2000, 0.1, 100]
 * /publish_world_models/meshes/west/pose/position: [0, -1000, -50]
 * /robot_description: <?xml version="1....
 * /rosdistro: melodic
 * /rosversion: 1.14.13
 * /twist_marker_server/link_name: /base_link
 * /twist_translate/cov_limits/velocity: 0.01
 * /twist_translate/fwd_vel/imax: 25.0
 * /twist_translate/fwd_vel/imin: -25.0
 * /twist_translate/fwd_vel/kd: 20
 * /twist_translate/fwd_vel/kf: 20
 * /twist_translate/fwd_vel/ki: 2
 * /twist_translate/fwd_vel/kp: 40
 * /twist_translate/max/bck_vel: 0.5
 * /twist_translate/max/fwd_vel: 3.0
 * /twist_translate/twist_cmd/timeout: 1.5
 * /twist_translate/yaw/imax: 0.0
 * /twist_translate/yaw/imin: 0.0
 * /twist_translate/yaw/kd: 0
 * /twist_translate/yaw/kf: 0
 * /twist_translate/yaw/ki: 0
 * /twist_translate/yaw/kp: 0.3
 * /twist_translate/yaw_rate/imax: 20.0
 * /twist_translate/yaw_rate/imin: -20.0
 * /twist_translate/yaw_rate/kd: 20
 * /twist_translate/yaw_rate/kf: 10
 * /twist_translate/yaw_rate/ki: 0.0
 * /twist_translate/yaw_rate/kp: 60
 * /use_sim_time: True

NODES
  /
    activate_control_service (heron_gazebo/activate_control_service)
    cmd_drive_to_thrusters (heron_gazebo/cmd_drive_translate)
    controller (heron_controller/controller)
    ekf_localization_node (robot_localization/ekf_localization_node)
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    imu_filter_madgwick (imu_filter_madgwick/imu_filter_node)
    mag_translation (heron_gazebo/vector3_to_magnetic_field)
    navsat_transform_node (robot_localization/navsat_transform_node)
    navsat_vel_cov (heron_control/vel_cov)
    navvel_translate (heron_gazebo/navsat_vel_translate)
    publish_world_models (uuv_assistants/publish_world_models.py)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rpy_translator (heron_gazebo/rpy_translator)
    twist_marker_server (interactive_marker_twist_server/marker_server)
    twist_translate (heron_gazebo/twist_translate)
    urdf_spawner (gazebo_ros/spawn_model)
    world_ned_frame_publisher (tf2_ros/static_transform_publisher)

auto-starting new master
process[master]: started with pid [48]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 2421ff3c-35a9-11ee-8419-7a77e1da858a
process[rosout-1]: started with pid [59]
started core service [/rosout]
process[gazebo-2]: started with pid [66]
Gazebo multi-robot simulator, version 9.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[ INFO] [1691471109.016573424]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1691471109.018205083]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.17.0.1
[Err] [RenderEngine.cc:725] Can't open display: 
[Wrn] [RenderEngine.cc:93] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:293] Cannot initialize render engine since render path type is NONE. Ignore this warning ifrendering has been turned off on purpose.
Warning [parser.cc:626] Can not find the XML attribute 'version' in sdf XML tag for model: NED frame. Please specify the SDF protocol supported in the model configuration file. The first sdf tag in the config file will be used 
process[gazebo_gui-3]: started with pid [71]
ALSA lib confmisc.c:767:(parse_card) cannot find card '0'
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_card_driver returned error: No such file or directory
ALSA lib confmisc.c:392:(snd_func_concat) error evaluating strings
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_concat returned error: No such file or directory
ALSA lib confmisc.c:1246:(snd_func_refer) error evaluating name
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_refer returned error: No such file or directory
ALSA lib conf.c:5007:(snd_config_expand) Evaluate error: No such file or directory
ALSA lib pcm.c:2495:(snd_pcm_open_noupdate) Unknown PCM default
AL lib: (EE) ALCplaybackAlsa_open: Could not open playback device 'default': No such file or directory
[Err] [OpenAL.cc:84] Unable to open audio device[default]
 Audio will be disabled.
[ INFO] [1691471109.245959177]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [1691471109.267339256]: Physics dynamic reconfigure ready.
Gazebo multi-robot simulator, version 9.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Wrn] [GuiIface.cc:199] g/gui-plugin is really loading a SystemPlugin. To load a GUI plugin please use --gui-client-plugin 
[ INFO] [1691471109.456785255]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1691471109.458984135]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.17.0.1
[Wrn] [GuiIface.cc:297] Couldn't locate specified .ini. Creating file at "/root/.gazebo/gui.ini"
[Err] [RenderEngine.cc:725] Can't open display: 
[Wrn] [RenderEngine.cc:93] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:293] Cannot initialize render engine since render path type is NONE. Ignore this warning ifrendering has been turned off on purpose.
[Wrn] [GuiIface.cc:120] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
[Wrn] [GuiIface.cc:120] QXcbConnection: Could not connect to display
[Err] [GuiIface.cc:124] Could not connect to any X display.
[Msg] Loading underwater world...
[Msg] Current velocity [m/s] Gauss-Markov process model:
[Msg]   Mean = 0
        Min. Limit = 0
        Max. Limit = 5
        Mu = 0
        Noise Amp. = 0
[Msg] Current velocity horizontal angle [rad] Gauss-Markov process model:
[Msg]   Mean = 0
        Min. Limit = -3.14159
        Max. Limit = 3.14159
        Mu = 0
        Noise Amp. = 0
[Msg] Current velocity horizontal angle [rad] Gauss-Markov process model:
[Msg]   Mean = 0
        Min. Limit = -3.14159
        Max. Limit = 3.14159
        Mu = 0
        Noise Amp. = 0
[Msg] Current velocity topic name: hydrodynamics/current_velocity
[Msg] Underwater current plugin loaded!
        WARNING: Current velocity calculated in the ENU frame
[Msg] UnderwaterCurrentROSPlugin::namespace=hydrodynamics
process[world_ned_frame_publisher-4]: started with pid [140]
[Msg] Spherical coordinates reference=
        - Latitude [degrees]=56.719
        - Longitude [degrees]=3.51562
        - Altitude [m]=0
process[publish_world_models-5]: started with pid [260]
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
terminate called recursively
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted
[gazebo_gui-3] process has died [pid 71, exit code 134, cmd /opt/ros/melodic/lib/gazebo_ros/gzclient --verbose __name:=gazebo_gui __log:=/root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/gazebo_gui-3.log].
log file: /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/gazebo_gui-3*.log
Start publishing vehicle footprints to RViz
process[cmd_drive_to_thrusters-6]: started with pid [281]
New model being published: north
         Position: [1000, 0, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: west
         Position: [0, -1000, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: heightmap
         Position: [0.0, 0.0, -95.0]
         Orientation: [0.0, 0.0, 0.0, 1.0]
         Scale: [1, 1, 1]
New model being published: seafloor
         Position: [0, 0, -100]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: east
         Position: [0, 1000, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: south
         Position: [-1000, 0, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
process[imu_filter_madgwick-7]: started with pid [287]
process[mag_translation-8]: started with pid [305]
process[controller-9]: started with pid [313]
process[rpy_translator-10]: started with pid [323]
process[urdf_spawner-11]: started with pid [336]
process[twist_marker_server-12]: started with pid [339]
[INFO] [1691471112.722819, 0.000000]: Loading model XML from ros parameter /robot_description
[INFO] [1691471112.729492, 0.000000]: Waiting for service /gazebo/spawn_urdf_model
[INFO] [1691471112.733261, 0.000000]: Calling service /gazebo/spawn_urdf_model
Warning [parser_urdf.cc:1115] multiple inconsistent <gravity> exists due to fixed joint reduction overwriting previous value [1] with [true].
[INFO] [1691471112.783551, 3.208000]: Spawn status: SpawnModel: Successfully spawned entity
Registered HydrodynamicModel type fossen
Registered HydrodynamicModel type sphere
Registered HydrodynamicModel type cylinder
Registered HydrodynamicModel type spheroid
Registered HydrodynamicModel type box
[Msg] Subscribing to current velocity topic: hydrodynamics/current_velocity
[Msg] Name of the BASE_LINK: base_link
[Msg] Surface vessel parameters
[Msg]   Metacentric width [m]=0.1
[Msg]   Metacentric length [m]=0.1
[Msg]   Submerged height [m]=0.02
[Msg] New bounding box for base_link::Min[-0.5 -0.675 -0.16] Max[0.5 0.675 0.16]
heron::base_link::added_mass
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
heron::base_link::scaling_added_mass
heron::base_link::offset_added_mass
heron::base_link::linear_damping
      -16.45           0           0           0           0           0
           0    -15.7978           0           0           0           0
           0           0        -100           0           0           0
           0           0           0         -13           0           0
           0           0           0           0         -13           0
           0           0           0           0           0          -6
heron::base_link::linear_damping_forward_speed
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
heron::base_link::quadratic_damping
      -2.942           0           0           0           0           0
           0    -2.76172           0           0           0           0
           0           0         -10           0           0           0
           0           0           0          -5           0           0
           0           0           0           0          -5           0
           0           0           0           0           0          -5
heron::base_link::scaling_damping
heron::base_link::offset_linear_damping
heron::base_link::offset_lin_forward_speed_damping
heron::base_link::offset_nonlin_damping
heron::base_link::volume
        0.13 m^3
heron::base_link::scaling_volume
Registered ThrusterDynamics type ZeroOrder
Registered ThrusterDynamics type FirstOrder
Registered ThrusterDynamics type Yoerger
Registered ThrusterDynamics type Bessa
Registered ConversionFunction type Basic
Registered ConversionFunction type Bessa
Registered ConversionFunction type LinearInterp
[Msg] ConversionFunctionLinearInterp::Create conversion function
[Msg]   - Input values:
-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1 
[Msg]   - Output values:
-19.88 -16.52 -12.6 -5.6 -1.4 0 2.24 9.52 21.28 28 33.6 
[Msg] Thruster #0 initialized
        - Link: thruster_0
        - Robot model: heron
        - Input command topic: /heron/thrusters/0/input
        - Thrust output topic: /heron/thrusters/0/thrust
[Msg] ConversionFunctionLinearInterp::Create conversion function
[Msg]   - Input values:
-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1 
[Msg]   - Output values:
-19.88 -16.52 -12.6 -5.6 -1.4 0 2.24 9.52 21.28 28 33.6 
[Msg] Thruster #1 initialized
        - Link: thruster_1
        - Robot model: heron
        - Input command topic: /heron/thrusters/1/input
        - Thrust output topic: /heron/thrusters/1/thrust
[Msg] JointStatePublisher::robotNamespace=/
[Msg] JointStatePublisher::Retrieving moving joints:
[Msg]   - thruster_0_joint
[Msg]   - thruster_1_joint
[Msg] base_link::waterLevelPlaneArea = 1.35
[urdf_spawner-11] process has finished cleanly
log file: /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/urdf_spawner-11*.log
process[twist_translate-13]: started with pid [350]
process[navvel_translate-14]: started with pid [375]
process[activate_control_service-15]: started with pid [391]
process[ekf_localization_node-16]: started with pid [405]
[activate_control_service-15] process has finished cleanly
log file: /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/activate_control_service-15*.log
[ WARN] [1691471114.368542756, 4.664000000]: Failed to meet update rate! Took 4.6639999999999997016
[ WARN] [1691471114.369603883, 4.664000000]: Failed to meet update rate! Took 4.6139999999999998792
[ WARN] [1691471114.384349559, 4.680000000]: Could not obtain transform from imu_link to base_link. Error was "base_link" passed to lookupTransform argument target_frame does not exist. 

process[navsat_transform_node-17]: started with pid [416]
[ INFO] [1691471114.535510104]: Waiting for valid clock time...
[ INFO] [1691471114.778880796, 5.074000000]: Valid clock time received. Starting node.
[ WARN] [1691471114.784941950, 5.074000000]: navsat_transform, Parameter 'broadcast_utm_transform' has been deprecated. Please use'broadcast_cartesian_transform' instead.
[ WARN] [1691471114.785307659, 5.074000000]: navsat_transform, Parameter 'broadcast_utm_transform_as_parent_frame' has been deprecated. Please use'broadcast_cartesian_transform_as_parent_frame' instead.
process[navsat_vel_cov-18]: started with pid [424]
[ INFO] [1691471115.030404795, 5.326000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.030461596, 5.326000000]: Datum UTM coordinate is (492818.438686, 5527517.135775) zone 32
[ INFO] [1691471115.054168400, 5.350000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.054224101, 5.350000000]: Datum UTM coordinate is (492818.438687, 5527517.135776) zone 32
[ INFO] [1691471115.103774262, 5.400000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.103829763, 5.400000000]: Datum UTM coordinate is (492818.438688, 5527517.135776) zone 32
[ INFO] [1691471115.127422763, 5.424000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.127475465, 5.424000000]: Datum UTM coordinate is (492818.438688, 5527517.135776) zone 32
[ INFO] [1691471115.153305222, 5.450000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.153361223, 5.450000000]: Datum UTM coordinate is (492818.438689, 5527517.135776) zone 32
[ INFO] [1691471115.177238731, 5.474000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.177294632, 5.474000000]: Datum UTM coordinate is (492818.438690, 5527517.135775) zone 32
[ INFO] [1691471115.203280693, 5.500000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.203338495, 5.500000000]: Datum UTM coordinate is (492818.438690, 5527517.135776) zone 32
[ INFO] [1691471115.227276904, 5.524000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.227333205, 5.524000000]: Datum UTM coordinate is (492818.438691, 5527517.135776) zone 32
[ INFO] [1691471115.229370357, 5.526000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.229423558, 5.526000000]: Datum UTM coordinate is (492818.438691, 5527517.135776) zone 32
[ INFO] [1691471115.253119661, 5.550000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.253174062, 5.550000000]: Datum UTM coordinate is (492818.438692, 5527517.135776) zone 32
process[robot_state_publisher-19]: started with pid [432]
[ INFO] [1691471115.303326838, 5.600000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.303392340, 5.600000000]: Datum UTM coordinate is (492818.438692, 5527517.135776) zone 32
[ INFO] [1691471115.353184207, 5.650000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.353242708, 5.650000000]: Datum UTM coordinate is (492818.438694, 5527517.135775) zone 32
[ INFO] [1691471115.377152217, 5.674000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.377207118, 5.674000000]: Datum UTM coordinate is (492818.438694, 5527517.135775) zone 32
[ INFO] [1691471115.403278181, 5.700000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.403335783, 5.700000000]: Datum UTM coordinate is (492818.438695, 5527517.135775) zone 32
[ INFO] [1691471115.427715703, 5.724000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.427780505, 5.724000000]: Datum UTM coordinate is (492818.438695, 5527517.135775) zone 32
[ INFO] [1691471115.429651252, 5.726000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.429718154, 5.726000000]: Datum UTM coordinate is (492818.438695, 5527517.135775) zone 32
[ INFO] [1691471115.453783266, 5.750000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.453838068, 5.750000000]: Datum UTM coordinate is (492818.438696, 5527517.135775) zone 32
[ INFO] [1691471115.477752476, 5.774000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.477808377, 5.774000000]: Datum UTM coordinate is (492818.438697, 5527517.135775) zone 32
[ INFO] [1691471115.479796528, 5.776000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.479850129, 5.776000000]: Datum UTM coordinate is (492818.438697, 5527517.135775) zone 32
[ INFO] [1691471115.503992444, 5.800000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.504049145, 5.800000000]: Datum UTM coordinate is (492818.438697, 5527517.135775) zone 32
[ INFO] [1691471115.553845773, 5.850000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.553902975, 5.850000000]: Datum UTM coordinate is (492818.438698, 5527517.135775) zone 32
[ INFO] [1691471115.568589149, 5.864000000]: Initial odometry pose is Origin: (8.9722170733370126368e-07 1.3144318502239113455e-07 0)
Rotation (RPY): (0, -0, 0.0032382941424533894306)

[ INFO] [1691471115.578027289, 5.874000000]: Corrected for magnetic declination of 0.000000, user-specified offset of 0.000000 and meridian convergence of -0.001335. Transform heading factor is now 0.001903
[ INFO] [1691471115.578087590, 5.874000000]: Transform world frame pose is: Origin: (8.9722170733370126368e-07 1.3144318502239113455e-07 0)
Rotation (RPY): (0, -0, 0.0032382941424533894306)

[ INFO] [1691471115.578150692, 5.874000000]: World frame->cartesian transform is Origin: (-485438.52890009403927 -5528170.0920567484573 -0.068063249935010464164)
Rotation (RPY): (0, -0, 0.0013350402698103301224)

[ WARN] [1691471115.628179865, 5.924000000]: Transform from base_link to odom was unavailable for the time requested. Using latest instead docker run -it --rm --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="${XAUTHORITY}:/root/.Xauthority:rw" --name="heron-sim-basic" --privileged docker_basic 
... logging to /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/roslaunch-docker-desktop-1.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://docker-desktop:33755/

SUMMARY
========

PARAMETERS
 * /cmd_drive_to_thrusters/namespace: 
 * /controller/fwd_vel/imax: 25.0     
 * /controller/fwd_vel/imin: -25.0    
 * /controller/fwd_vel/kd: 20
 * /controller/fwd_vel/kf: 20
 * /controller/fwd_vel/ki: 2
 * /controller/fwd_vel/kp: 40
 * /controller/max/bck_vel: 0.5
 * /controller/max/fwd_vel: 3.0
 * /controller/twist_cmd/timeout: 1.5
 * /controller/yaw/imax: 0.0
 * /controller/yaw/imin: 0.0
 * /controller/yaw/kd: 0
 * /controller/yaw/kf: 0
 * /controller/yaw/ki: 0
 * /controller/yaw/kp: 0.3
 * /controller/yaw_rate/imax: 20.0
 * /controller/yaw_rate/imin: -20.0
 * /controller/yaw_rate/kd: 20
 * /controller/yaw_rate/kf: 10
 * /controller/yaw_rate/ki: 0.0
 * /controller/yaw_rate/kp: 60
 * /ekf_localization_node/base_link_frame: base_link
 * /ekf_localization_node/frequency: 20
 * /ekf_localization_node/imu0: imu/data
 * /ekf_localization_node/imu0_config: [False, False, Fa...
 * /ekf_localization_node/imu0_differential: False
 * /ekf_localization_node/imu0_nodelay: True
 * /ekf_localization_node/imu0_queue_size: 10
 * /ekf_localization_node/imu0_relative: False
 * /ekf_localization_node/imu0_remove_gravitational_acceleration: True
 * /ekf_localization_node/map_frame: map
 * /ekf_localization_node/odom0: odometry/gps
 * /ekf_localization_node/odom0_config: [True, True, Fals...
 * /ekf_localization_node/odom0_differential: False
 * /ekf_localization_node/odom0_nodelay: True
 * /ekf_localization_node/odom0_queue_size: 10
 * /ekf_localization_node/odom0_relative: False
 * /ekf_localization_node/odom_frame: odom
 * /ekf_localization_node/process_noise_covariance: [0.1, 0, 0, 0, 0,...
 * /ekf_localization_node/sensor_timeout: 0.1
 * /ekf_localization_node/transform_time_offset: 0.0
 * /ekf_localization_node/transform_timeout: 0.0
 * /ekf_localization_node/twist0: navsat/vel_cov
 * /ekf_localization_node/twist0_config: [False, False, Fa...
 * /ekf_localization_node/twist0_differential: False
 * /ekf_localization_node/twist0_nodelay: True
 * /ekf_localization_node/twist0_queue_size: 10
 * /ekf_localization_node/twist0_relative: False
 * /ekf_localization_node/two_d_mode: True
 * /ekf_localization_node/use_control: False
 * /ekf_localization_node/world_frame: odom
 * /gazebo/enable_ros_network: True
 * /imu_filter_madgwick/publish_debug_topics: True
 * /imu_filter_madgwick/publish_tf: False
 * /imu_filter_madgwick/use_mag: True
 * /imu_filter_madgwick/use_magnetic_field_msg: True
 * /imu_filter_madgwick/world_frame: enu
 * /navsat_transform_node/broadcast_utm_transform: True
 * /navsat_transform_node/broadcast_utm_transform_as_parent_frame: True
 * /navsat_transform_node/datum: [43.4720948, -80....
 * /navsat_transform_node/delay: 0.0
 * /navsat_transform_node/frequency: 20
 * /navsat_transform_node/magnetic_declination_radians: 0
 * /navsat_transform_node/publish_filtered_gps: False
 * /navsat_transform_node/use_odometry_yaw: True
 * /navsat_transform_node/wait_for_datum: False
 * /navsat_transform_node/yaw_offset: 0.0
 * /navsat_transform_node/zero_altitude: False
 * /publish_world_models/meshes/east/plane: [2000, 0.1, 100]
 * /publish_world_models/meshes/east/pose/position: [0, 1000, -50]
 * /publish_world_models/meshes/heightmap/mesh: package://uuv_gaz...
 * /publish_world_models/meshes/heightmap/model: sand_heightmap
 * /publish_world_models/meshes/north/plane: [0.1, 2000, 100]
 * /publish_world_models/meshes/north/pose/position: [1000, 0, -50]
 * /publish_world_models/meshes/seafloor/plane: [2000, 2000, 0.1]
 * /publish_world_models/meshes/seafloor/pose/position: [0, 0, -100]
 * /publish_world_models/meshes/south/plane: [0.1, 2000, 100]
 * /publish_world_models/meshes/south/pose/position: [-1000, 0, -50]
 * /publish_world_models/meshes/west/plane: [2000, 0.1, 100]
 * /publish_world_models/meshes/west/pose/position: [0, -1000, -50]
 * /robot_description: <?xml version="1....
 * /rosdistro: melodic
 * /rosversion: 1.14.13
 * /twist_marker_server/link_name: /base_link
 * /twist_translate/cov_limits/velocity: 0.01
 * /twist_translate/fwd_vel/imax: 25.0
 * /twist_translate/fwd_vel/imin: -25.0
 * /twist_translate/fwd_vel/kd: 20
 * /twist_translate/fwd_vel/kf: 20
 * /twist_translate/fwd_vel/ki: 2
 * /twist_translate/fwd_vel/kp: 40
 * /twist_translate/max/bck_vel: 0.5
 * /twist_translate/max/fwd_vel: 3.0
 * /twist_translate/twist_cmd/timeout: 1.5
 * /twist_translate/yaw/imax: 0.0
 * /twist_translate/yaw/imin: 0.0
 * /twist_translate/yaw/kd: 0
 * /twist_translate/yaw/kf: 0
 * /twist_translate/yaw/ki: 0
 * /twist_translate/yaw/kp: 0.3
 * /twist_translate/yaw_rate/imax: 20.0
 * /twist_translate/yaw_rate/imin: -20.0
 * /twist_translate/yaw_rate/kd: 20
 * /twist_translate/yaw_rate/kf: 10
 * /twist_translate/yaw_rate/ki: 0.0
 * /twist_translate/yaw_rate/kp: 60
 * /use_sim_time: True

NODES
  /
    activate_control_service (heron_gazebo/activate_control_service)
    cmd_drive_to_thrusters (heron_gazebo/cmd_drive_translate)
    controller (heron_controller/controller)
    ekf_localization_node (robot_localization/ekf_localization_node)
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    imu_filter_madgwick (imu_filter_madgwick/imu_filter_node)
    mag_translation (heron_gazebo/vector3_to_magnetic_field)
    navsat_transform_node (robot_localization/navsat_transform_node)
    navsat_vel_cov (heron_control/vel_cov)
    navvel_translate (heron_gazebo/navsat_vel_translate)
    publish_world_models (uuv_assistants/publish_world_models.py)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rpy_translator (heron_gazebo/rpy_translator)
    twist_marker_server (interactive_marker_twist_server/marker_server)
    twist_translate (heron_gazebo/twist_translate)
    urdf_spawner (gazebo_ros/spawn_model)
    world_ned_frame_publisher (tf2_ros/static_transform_publisher)

auto-starting new master
process[master]: started with pid [48]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 2421ff3c-35a9-11ee-8419-7a77e1da858a
process[rosout-1]: started with pid [59]
started core service [/rosout]
process[gazebo-2]: started with pid [66]
Gazebo multi-robot simulator, version 9.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[ INFO] [1691471109.016573424]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1691471109.018205083]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.17.0.1
[Err] [RenderEngine.cc:725] Can't open display: 
[Wrn] [RenderEngine.cc:93] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:293] Cannot initialize render engine since render path type is NONE. Ignore this warning ifrendering has been turned off on purpose.
Warning [parser.cc:626] Can not find the XML attribute 'version' in sdf XML tag for model: NED frame. Please specify the SDF protocol supported in the model configuration file. The first sdf tag in the config file will be used 
process[gazebo_gui-3]: started with pid [71]
ALSA lib confmisc.c:767:(parse_card) cannot find card '0'
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_card_driver returned error: No such file or directory
ALSA lib confmisc.c:392:(snd_func_concat) error evaluating strings
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_concat returned error: No such file or directory
ALSA lib confmisc.c:1246:(snd_func_refer) error evaluating name
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_refer returned error: No such file or directory
ALSA lib conf.c:5007:(snd_config_expand) Evaluate error: No such file or directory
ALSA lib pcm.c:2495:(snd_pcm_open_noupdate) Unknown PCM default
AL lib: (EE) ALCplaybackAlsa_open: Could not open playback device 'default': No such file or directory
[Err] [OpenAL.cc:84] Unable to open audio device[default]
 Audio will be disabled.
[ INFO] [1691471109.245959177]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [1691471109.267339256]: Physics dynamic reconfigure ready.
Gazebo multi-robot simulator, version 9.0.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Wrn] [GuiIface.cc:199] g/gui-plugin is really loading a SystemPlugin. To load a GUI plugin please use --gui-client-plugin 
[ INFO] [1691471109.456785255]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1691471109.458984135]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 172.17.0.1
[Wrn] [GuiIface.cc:297] Couldn't locate specified .ini. Creating file at "/root/.gazebo/gui.ini"
[Err] [RenderEngine.cc:725] Can't open display: 
[Wrn] [RenderEngine.cc:93] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:293] Cannot initialize render engine since render path type is NONE. Ignore this warning ifrendering has been turned off on purpose.
[Wrn] [GuiIface.cc:120] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
[Wrn] [GuiIface.cc:120] QXcbConnection: Could not connect to display
[Err] [GuiIface.cc:124] Could not connect to any X display.
[Msg] Loading underwater world...
[Msg] Current velocity [m/s] Gauss-Markov process model:
[Msg]   Mean = 0
        Min. Limit = 0
        Max. Limit = 5
        Mu = 0
        Noise Amp. = 0
[Msg] Current velocity horizontal angle [rad] Gauss-Markov process model:
[Msg]   Mean = 0
        Min. Limit = -3.14159
        Max. Limit = 3.14159
        Mu = 0
        Noise Amp. = 0
[Msg] Current velocity horizontal angle [rad] Gauss-Markov process model:
[Msg]   Mean = 0
        Min. Limit = -3.14159
        Max. Limit = 3.14159
        Mu = 0
        Noise Amp. = 0
[Msg] Current velocity topic name: hydrodynamics/current_velocity
[Msg] Underwater current plugin loaded!
        WARNING: Current velocity calculated in the ENU frame
[Msg] UnderwaterCurrentROSPlugin::namespace=hydrodynamics
process[world_ned_frame_publisher-4]: started with pid [140]
[Msg] Spherical coordinates reference=
        - Latitude [degrees]=56.719
        - Longitude [degrees]=3.51562
        - Altitude [m]=0
process[publish_world_models-5]: started with pid [260]
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
terminate called recursively
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted
[gazebo_gui-3] process has died [pid 71, exit code 134, cmd /opt/ros/melodic/lib/gazebo_ros/gzclient --verbose __name:=gazebo_gui __log:=/root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/gazebo_gui-3.log].
log file: /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/gazebo_gui-3*.log
Start publishing vehicle footprints to RViz
process[cmd_drive_to_thrusters-6]: started with pid [281]
New model being published: north
         Position: [1000, 0, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: west
         Position: [0, -1000, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: heightmap
         Position: [0.0, 0.0, -95.0]
         Orientation: [0.0, 0.0, 0.0, 1.0]
         Scale: [1, 1, 1]
New model being published: seafloor
         Position: [0, 0, -100]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: east
         Position: [0, 1000, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
New model being published: south
         Position: [-1000, 0, -50]
         Orientation: [0, 0, 0, 1]
         Scale: [1, 1, 1]
process[imu_filter_madgwick-7]: started with pid [287]
process[mag_translation-8]: started with pid [305]
process[controller-9]: started with pid [313]
process[rpy_translator-10]: started with pid [323]
process[urdf_spawner-11]: started with pid [336]
process[twist_marker_server-12]: started with pid [339]
[INFO] [1691471112.722819, 0.000000]: Loading model XML from ros parameter /robot_description
[INFO] [1691471112.729492, 0.000000]: Waiting for service /gazebo/spawn_urdf_model
[INFO] [1691471112.733261, 0.000000]: Calling service /gazebo/spawn_urdf_model
Warning [parser_urdf.cc:1115] multiple inconsistent <gravity> exists due to fixed joint reduction overwriting previous value [1] with [true].
[INFO] [1691471112.783551, 3.208000]: Spawn status: SpawnModel: Successfully spawned entity
Registered HydrodynamicModel type fossen
Registered HydrodynamicModel type sphere
Registered HydrodynamicModel type cylinder
Registered HydrodynamicModel type spheroid
Registered HydrodynamicModel type box
[Msg] Subscribing to current velocity topic: hydrodynamics/current_velocity
[Msg] Name of the BASE_LINK: base_link
[Msg] Surface vessel parameters
[Msg]   Metacentric width [m]=0.1
[Msg]   Metacentric length [m]=0.1
[Msg]   Submerged height [m]=0.02
[Msg] New bounding box for base_link::Min[-0.5 -0.675 -0.16] Max[0.5 0.675 0.16]
heron::base_link::added_mass
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
heron::base_link::scaling_added_mass
heron::base_link::offset_added_mass
heron::base_link::linear_damping
      -16.45           0           0           0           0           0
           0    -15.7978           0           0           0           0
           0           0        -100           0           0           0
           0           0           0         -13           0           0
           0           0           0           0         -13           0
           0           0           0           0           0          -6
heron::base_link::linear_damping_forward_speed
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
           0           0           0           0           0           0
heron::base_link::quadratic_damping
      -2.942           0           0           0           0           0
           0    -2.76172           0           0           0           0
           0           0         -10           0           0           0
           0           0           0          -5           0           0
           0           0           0           0          -5           0
           0           0           0           0           0          -5
heron::base_link::scaling_damping
heron::base_link::offset_linear_damping
heron::base_link::offset_lin_forward_speed_damping
heron::base_link::offset_nonlin_damping
heron::base_link::volume
        0.13 m^3
heron::base_link::scaling_volume
Registered ThrusterDynamics type ZeroOrder
Registered ThrusterDynamics type FirstOrder
Registered ThrusterDynamics type Yoerger
Registered ThrusterDynamics type Bessa
Registered ConversionFunction type Basic
Registered ConversionFunction type Bessa
Registered ConversionFunction type LinearInterp
[Msg] ConversionFunctionLinearInterp::Create conversion function
[Msg]   - Input values:
-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1 
[Msg]   - Output values:
-19.88 -16.52 -12.6 -5.6 -1.4 0 2.24 9.52 21.28 28 33.6 
[Msg] Thruster #0 initialized
        - Link: thruster_0
        - Robot model: heron
        - Input command topic: /heron/thrusters/0/input
        - Thrust output topic: /heron/thrusters/0/thrust
[Msg] ConversionFunctionLinearInterp::Create conversion function
[Msg]   - Input values:
-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1 
[Msg]   - Output values:
-19.88 -16.52 -12.6 -5.6 -1.4 0 2.24 9.52 21.28 28 33.6 
[Msg] Thruster #1 initialized
        - Link: thruster_1
        - Robot model: heron
        - Input command topic: /heron/thrusters/1/input
        - Thrust output topic: /heron/thrusters/1/thrust
[Msg] JointStatePublisher::robotNamespace=/
[Msg] JointStatePublisher::Retrieving moving joints:
[Msg]   - thruster_0_joint
[Msg]   - thruster_1_joint
[Msg] base_link::waterLevelPlaneArea = 1.35
[urdf_spawner-11] process has finished cleanly
log file: /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/urdf_spawner-11*.log
process[twist_translate-13]: started with pid [350]
process[navvel_translate-14]: started with pid [375]
process[activate_control_service-15]: started with pid [391]
process[ekf_localization_node-16]: started with pid [405]
[activate_control_service-15] process has finished cleanly
log file: /root/.ros/log/2421ff3c-35a9-11ee-8419-7a77e1da858a/activate_control_service-15*.log
[ WARN] [1691471114.368542756, 4.664000000]: Failed to meet update rate! Took 4.6639999999999997016
[ WARN] [1691471114.369603883, 4.664000000]: Failed to meet update rate! Took 4.6139999999999998792
[ WARN] [1691471114.384349559, 4.680000000]: Could not obtain transform from imu_link to base_link. Error was "base_link" passed to lookupTransform argument target_frame does not exist. 

process[navsat_transform_node-17]: started with pid [416]
[ INFO] [1691471114.535510104]: Waiting for valid clock time...
[ INFO] [1691471114.778880796, 5.074000000]: Valid clock time received. Starting node.
[ WARN] [1691471114.784941950, 5.074000000]: navsat_transform, Parameter 'broadcast_utm_transform' has been deprecated. Please use'broadcast_cartesian_transform' instead.
[ WARN] [1691471114.785307659, 5.074000000]: navsat_transform, Parameter 'broadcast_utm_transform_as_parent_frame' has been deprecated. Please use'broadcast_cartesian_transform_as_parent_frame' instead.
process[navsat_vel_cov-18]: started with pid [424]
[ INFO] [1691471115.030404795, 5.326000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.030461596, 5.326000000]: Datum UTM coordinate is (492818.438686, 5527517.135775) zone 32
[ INFO] [1691471115.054168400, 5.350000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.054224101, 5.350000000]: Datum UTM coordinate is (492818.438687, 5527517.135776) zone 32
[ INFO] [1691471115.103774262, 5.400000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.103829763, 5.400000000]: Datum UTM coordinate is (492818.438688, 5527517.135776) zone 32
[ INFO] [1691471115.127422763, 5.424000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.127475465, 5.424000000]: Datum UTM coordinate is (492818.438688, 5527517.135776) zone 32
[ INFO] [1691471115.153305222, 5.450000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.153361223, 5.450000000]: Datum UTM coordinate is (492818.438689, 5527517.135776) zone 32
[ INFO] [1691471115.177238731, 5.474000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.177294632, 5.474000000]: Datum UTM coordinate is (492818.438690, 5527517.135775) zone 32
[ INFO] [1691471115.203280693, 5.500000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.203338495, 5.500000000]: Datum UTM coordinate is (492818.438690, 5527517.135776) zone 32
[ INFO] [1691471115.227276904, 5.524000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.227333205, 5.524000000]: Datum UTM coordinate is (492818.438691, 5527517.135776) zone 32
[ INFO] [1691471115.229370357, 5.526000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.229423558, 5.526000000]: Datum UTM coordinate is (492818.438691, 5527517.135776) zone 32
[ INFO] [1691471115.253119661, 5.550000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140064)
[ INFO] [1691471115.253174062, 5.550000000]: Datum UTM coordinate is (492818.438692, 5527517.135776) zone 32
process[robot_state_publisher-19]: started with pid [432]
[ INFO] [1691471115.303326838, 5.600000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.303392340, 5.600000000]: Datum UTM coordinate is (492818.438692, 5527517.135776) zone 32
[ INFO] [1691471115.353184207, 5.650000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.353242708, 5.650000000]: Datum UTM coordinate is (492818.438694, 5527517.135775) zone 32
[ INFO] [1691471115.377152217, 5.674000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.377207118, 5.674000000]: Datum UTM coordinate is (492818.438694, 5527517.135775) zone 32
[ INFO] [1691471115.403278181, 5.700000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.403335783, 5.700000000]: Datum UTM coordinate is (492818.438695, 5527517.135775) zone 32
[ INFO] [1691471115.427715703, 5.724000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.427780505, 5.724000000]: Datum UTM coordinate is (492818.438695, 5527517.135775) zone 32
[ INFO] [1691471115.429651252, 5.726000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.429718154, 5.726000000]: Datum UTM coordinate is (492818.438695, 5527517.135775) zone 32
[ INFO] [1691471115.453783266, 5.750000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.453838068, 5.750000000]: Datum UTM coordinate is (492818.438696, 5527517.135775) zone 32
[ INFO] [1691471115.477752476, 5.774000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.477808377, 5.774000000]: Datum UTM coordinate is (492818.438697, 5527517.135775) zone 32
[ INFO] [1691471115.479796528, 5.776000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.479850129, 5.776000000]: Datum UTM coordinate is (492818.438697, 5527517.135775) zone 32
[ INFO] [1691471115.503992444, 5.800000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.504049145, 5.800000000]: Datum UTM coordinate is (492818.438697, 5527517.135775) zone 32
[ INFO] [1691471115.553845773, 5.850000000]: Datum (latitude, longitude, altitude) is (49.900000, 8.900000, 0.140063)
[ INFO] [1691471115.553902975, 5.850000000]: Datum UTM coordinate is (492818.438698, 5527517.135775) zone 32
[ INFO] [1691471115.568589149, 5.864000000]: Initial odometry pose is Origin: (8.9722170733370126368e-07 1.3144318502239113455e-07 0)
Rotation (RPY): (0, -0, 0.0032382941424533894306)

[ INFO] [1691471115.578027289, 5.874000000]: Corrected for magnetic declination of 0.000000, user-specified offset of 0.000000 and meridian convergence of -0.001335. Transform heading factor is now 0.001903
[ INFO] [1691471115.578087590, 5.874000000]: Transform world frame pose is: Origin: (8.9722170733370126368e-07 1.3144318502239113455e-07 0)
Rotation (RPY): (0, -0, 0.0032382941424533894306)

[ INFO] [1691471115.578150692, 5.874000000]: World frame->cartesian transform is Origin: (-485438.52890009403927 -5528170.0920567484573 -0.068063249935010464164)
Rotation (RPY): (0, -0, 0.0013350402698103301224)

[ WARN] [1691471115.628179865, 5.924000000]: Transform from base_link to odom was unavailable for the time requested. Using latest instead

```