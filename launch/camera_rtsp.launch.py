"""Camera-specific capture profiles sharing the package's RTSP encoder path."""

from pathlib import Path
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


PROFILES = {
    'zed': {
        'package': 'v4l2_camera', 'plugin': 'v4l2_camera::V4L2Camera',
        'executable': 'v4l2_camera_node', 'camera_name': 'zed_capture',
        'width': '2560', 'height': '720', 'framerate': '30', 'composed': 'false',
    },
    'realsense': {
        'package': 'realsense2_camera', 'plugin': 'realsense2_camera::RealSenseNodeFactory',
        'executable': 'realsense2_camera_node', 'camera_name': 'camera',
        'width': '1280', 'height': '720', 'framerate': '30', 'composed': 'true',
    },
    'lucid': {
        'package': 'lucid_camera_driver', 'plugin': 'Lucid_Driver::Lucid_Camera',
        'executable': 'lucid_camera_node', 'camera_name': 'lucid_capture',
        'width': '0', 'height': '0', 'framerate': '15', 'composed': 'true',
    },
}


def _boolean(name, value):
    if value.lower() not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name} must be true or false')
    return value.lower() in ('true', '1')


def _configuration(context):
    """Resolve typed settings without opening hardware or importing a vendor SDK."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    kind = value('camera_type')
    profile = PROFILES[kind]

    def setting(name):
        return value(name) or profile[name]

    namespace = '/' + (value('camera_namespace') or kind).strip('/')
    prefix = namespace.rstrip('/')
    raw, encoded, decoded = (f'{prefix}/{suffix}' for suffix in (
        'image_raw', 'image_h264/rtsp', 'image_decoded'))
    name = setting('camera_name')
    fps, width, height = (int(setting(key)) for key in ('framerate', 'width', 'height'))
    bitrate, port = int(value('bitrate')), int(value('rtsp_port'))
    if fps <= 0 or bitrate <= 0 or not 1 <= port <= 65535:
        raise ValueError('framerate and bitrate must be positive; rtsp_port must be in 1..65535')
    if kind != 'lucid' and min(width, height) <= 0:
        raise ValueError('width and height must be positive')
    if kind == 'lucid' and (width != 0 or height != 0):
        raise ValueError('Set Lucid ROI through configuration_file, not width/height')
    decoder_budget = float(value('max_latency'))
    timestamp_source = int(value('timestamp_source'))
    if not 0 <= decoder_budget <= 10 or timestamp_source not in (0, 1, 2):
        raise ValueError('max_latency must be in 0..10 seconds; timestamp_source must be 0, 1 or 2')

    frame_id = value('frame_id') or (
        f'{name}_color_optical_frame' if kind == 'realsense' else f'{kind}_camera_optical_frame')
    parameters = {'image_raw.enable_pub_plugins': ['image_transport/raw']}
    remappings = []
    if kind == 'zed':
        parameters.update({
            'video_device': value('video_device'), 'image_size': [width, height],
            'pixel_format': 'YUYV', 'output_encoding': value('output_encoding'),
            'camera_frame_id': frame_id,
            f'qos_overrides.{raw}.publisher.history': 'keep_last',
            f'qos_overrides.{raw}.publisher.depth': 1,
        })
    elif kind == 'realsense':
        parameters.update({
            'camera_name': name, 'serial_no': value('serial_number'),
            'enable_color': True, 'enable_depth': False,
            'enable_infra': False, 'enable_infra1': False, 'enable_infra2': False,
            'enable_gyro': False, 'enable_accel': False, 'enable_sync': False,
            'enable_rgbd': False, 'pointcloud.enable': False,
            'align_depth.enable': False,
            'rgb_camera.color_profile': f'{width},{height},{fps}',
            'rgb_camera.color_format': value('color_format'),
            'color_qos': 'SENSOR_DATA',
        })
        # The driver's color topic is private, including the node's name.
        remappings = [('~/color/image_raw', raw)]
    else:
        packet_size = int(value('stream_packet_size'))
        if packet_size < 0:
            raise ValueError('stream_packet_size must be nonnegative')
        parameters.update({
            'serial_number': value('serial_number'), 'frame_rate': float(fps),
            # Empty means retain the device's native PFNC format. The driver
            # derives the actual ROS encoding from the camera, not this string.
            'image_encoding': value('image_encoding'), 'frame_id': frame_id,
            'enable_ptp': _boolean('enable_ptp', value('enable_ptp')),
            'use_camera_time': _boolean('use_camera_time', value('use_camera_time')),
            'configuration_file': value('configuration_file'),
            'stream_auto_negotiate_packet_size': _boolean(
                'stream_auto_negotiate_packet_size', value('stream_auto_negotiate_packet_size')),
            'stream_packet_size': packet_size,
        })

    parameter_sources = [parameters]
    for key in ('driver_params_file', 'configuration_file'):
        path = value(key)
        if path and not Path(path).is_file():
            raise ValueError(f'{key} does not exist: {path}')
    if value('driver_params_file'):
        parameter_sources.append(value('driver_params_file'))

    encoder_parameters = {
        'in_transport': 'raw', 'out_transport': 'rtsp', 'out.rtsp.codec': 'H264',
        'out.rtsp.target_bitrate': bitrate, 'out.rtsp.expected_framerate': fps,
        'out.rtsp.keyframe_interval': 0, 'out.rtsp.intra_refresh': False,
        'out.rtsp.use_hw_encoder': True, 'out.rtsp.udp_port': port,
        'out.rtsp.udp_packet_size': 1396, 'out.rtsp.use_ip_multicast': False,
        f'qos_overrides.{raw}.subscription.reliability': 'best_effort',
        f'qos_overrides.{raw}.subscription.history': 'keep_last',
        f'qos_overrides.{raw}.subscription.depth': 1,
        f'qos_overrides.{raw}.subscription.durability': 'volatile',
    }
    return {
        'kind': kind, 'profile': profile, 'namespace': namespace, 'name': name,
        'raw': raw, 'encoded': encoded, 'decoded': decoded, 'fps': fps,
        'width': width, 'height': height, 'frame_id': frame_id,
        'parameters': parameter_sources, 'remappings': remappings,
        'encoder_parameters': encoder_parameters,
        'composed': _boolean('composed', setting('composed')),
        'decode': _boolean('decode', value('decode')),
        'rtp_over_tcp': _boolean('rtp_over_tcp', value('rtp_over_tcp')),
        'low_latency': _boolean('low_latency', value('low_latency')),
        'max_latency': decoder_budget, 'timestamp_source': timestamp_source,
    }


def _pipeline(context):
    config = _configuration(context)
    profile = config['profile']
    try:
        get_package_share_directory(profile['package'])
    except LookupError as error:
        raise RuntimeError(
            f"The {config['kind']} example requires the optional {profile['package']} driver; "
            'see docs/camera_profiles.rst') from error
    if config['kind'] == 'zed':
        device = config['parameters'][0]['video_device']
        if not Path(device).exists():
            raise RuntimeError(f'ZED capture device does not exist: {device}')
        if shutil.which('v4l2-ctl') is None:
            raise RuntimeError('The ZED example requires v4l2-ctl from v4l-utils')
        subprocess.run([
            'v4l2-ctl', '--device', device,
            '--set-fmt-video', f"width={config['width']},height={config['height']},pixelformat=YUYV",
            '--set-parm', str(config['fps']),
        ], check=True)

    source = dict(package=profile['package'], name=config['name'],
                  namespace=config['namespace'], parameters=config['parameters'],
                  remappings=config['remappings'])
    encoder_namespace = '' if config['kind'] == 'zed' else config['namespace']
    encoder = dict(package='image_transport', name='image_to_rtsp_h264',
                   namespace=encoder_namespace, parameters=[config['encoder_parameters']],
                   remappings=[('in', config['raw']), ('out/rtsp', config['encoded'])])
    if config['composed']:
        capture_and_encoder = [ComposableNodeContainer(
            name='zed_rtsp_container' if config['kind'] == 'zed' else 'rtsp_camera_container',
            namespace=encoder_namespace,
            package='rclcpp_components', executable='component_container',
            arguments=['--executor-type', 'multi-threaded'], output='screen',
            composable_node_descriptions=[
                ComposableNode(**source, plugin=profile['plugin'],
                               extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(**encoder, plugin='image_transport::Republisher',
                               extra_arguments=[{'use_intra_process_comms': True}]),
            ],
        )]
    else:
        capture_and_encoder = [
            Node(**source, executable=profile['executable'], output='screen'),
            Node(**encoder, executable='republish', output='screen'),
        ]

    actions = [LogInfo(msg=(f"{config['kind']}: {config['raw']} -> {config['encoded']} "
                           f"({config['fps']} fps; composed={config['composed']})")),
               *capture_and_encoder]
    if config['decode']:
        transport_launch = Path(get_package_share_directory('rtsp_image_transport')) / 'launch'
        decoder = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(transport_launch / 'rtsp_to_ros.launch.yaml')),
            launch_arguments={
                'input_topic': config['encoded'], 'output_topic': config['decoded'],
                'frame_id': config['frame_id'], 'timestamp_source': str(config['timestamp_source']),
                'rtp_over_tcp': str(config['rtp_over_tcp']).lower(),
                'low_latency': str(config['low_latency']).lower(),
                'max_latency': str(config['max_latency']),
            }.items(),
        )
        actions.append(decoder if config['kind'] == 'zed' else GroupAction([
            PushRosNamespace(config['namespace']), decoder,
        ]))
    return actions


def generate_launch_description():
    arguments = [DeclareLaunchArgument('camera_type', default_value='zed', choices=list(PROFILES))]
    settings = {
        'camera_namespace': ('', 'Blank selects /zed, /realsense or /lucid'),
        'camera_name': ('', 'Blank selects the camera profile node name'),
        'width': ('', 'ZED default 2560; RealSense 1280; Lucid uses device ROI'),
        'height': ('', 'ZED/RealSense default 720; Lucid uses device ROI'),
        'framerate': ('', 'Capture/encoder FPS: ZED/RealSense 30, Lucid 15'),
        'composed': ('', 'Share process and intra-process images; default false for ZED, true otherwise'),
        'bitrate': ('8000000', 'H.264 target bits per second'),
        'rtsp_port': ('8555', 'RTSP listening port'),
        'decode': ('true', 'Offer decoded ROS images on demand'),
        'rtp_over_tcp': ('false', 'Use TCP for the optional ROS decoder'),
        'low_latency': ('true', 'Minimize optional ROS decoder buffering'),
        'max_latency': ('0.2', 'Optional ROS decoder queue budget in seconds; 0 disables dropping'),
        'timestamp_source': ('1', 'Decoded image timestamps: 0 sender, 1 receiver, 2 automatic'),
        'frame_id': ('', 'Optical frame ID for the optional ROS decoder'),
        'driver_params_file': ('', 'Optional ROS driver YAML; overrides profile parameters'),
        'serial_number': ('', 'Optional RealSense or Lucid device serial, kept as a string'),
        'video_device': ('/dev/v4l/by-id/usb-Technologies__Inc._ZED_2-video-index0', 'ZED V4L2 path'),
        'output_encoding': ('rgb8', 'ZED image encoding; yuv422_yuy2 avoids RGB conversion'),
        'color_format': ('RGB8', 'RealSense color format; YUYV avoids RGB conversion when supported'),
        'image_encoding': ('', 'Lucid PFNC/ROS encoding; blank retains the camera native format'),
        'configuration_file': ('', 'Lucid GenICam lucid_params YAML for ROI/exposure/throughput'),
        'enable_ptp': ('false', 'Lucid PTP synchronization'),
        'use_camera_time': ('false', 'Lucid camera timestamps; requires a compatible clock'),
        'stream_auto_negotiate_packet_size': ('false', 'Lucid packet negotiation; explicit size otherwise'),
        'stream_packet_size': ('1400', 'Lucid GigE packet bytes including IP/UDP headers; 0 leaves it unchanged'),
    }
    return LaunchDescription(arguments + [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in settings.items()
    ] + [OpaqueFunction(function=_pipeline)])
