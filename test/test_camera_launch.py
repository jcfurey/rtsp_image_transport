"""Driver launch contracts tested without opening cameras or importing SDKs."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('camera_rtsp_launch', ROOT / 'launch/camera_rtsp.launch.py')
CAMERA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAMERA)


def context(kind, **overrides):
    result = LaunchContext()
    for entity in CAMERA.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(result)
    result.launch_configurations.update(camera_type=kind, **overrides)
    return result


class CameraLaunch(unittest.TestCase):
    def test_zed_preserves_existing_topics_and_defaults(self):
        config = CAMERA._configuration(context('zed'))
        self.assertEqual((config['width'], config['height'], config['fps']), (2560, 720, 30))
        self.assertEqual(config['raw'], '/zed/image_raw')
        self.assertEqual(config['encoded'], '/zed/image_h264/rtsp')
        self.assertEqual(config['decoded'], '/zed/image_decoded')
        self.assertEqual(config['parameters'][0]['output_encoding'], 'rgb8')
        self.assertFalse(config['composed'])

    def test_realsense_routes_private_color_topic_and_disables_unused_streams(self):
        config = CAMERA._configuration(context('realsense', serial_number='123456789', color_format='YUYV'))
        params = config['parameters'][0]
        self.assertTrue(config['composed'])
        self.assertEqual(config['remappings'], [('~/color/image_raw', '/realsense/image_raw')])
        self.assertEqual(params['serial_no'], '123456789')
        self.assertIsInstance(params['serial_no'], str)
        self.assertEqual(params['rgb_camera.color_profile'], '1280,720,30')
        self.assertEqual(params['rgb_camera.color_format'], 'YUYV')
        self.assertEqual(params['color_qos'], 'SENSOR_DATA')
        self.assertTrue(params['enable_color'])
        for name in ('enable_depth', 'enable_infra', 'enable_infra1', 'enable_infra2',
                     'enable_gyro', 'enable_accel', 'enable_sync', 'enable_rgbd',
                     'pointcloud.enable', 'align_depth.enable'):
            self.assertIs(params[name], False, name)

    def test_lucid_keeps_native_encoding_and_uses_typed_driver_parameters(self):
        config = CAMERA._configuration(context('lucid', serial_number='220903154'))
        params = config['parameters'][0]
        self.assertEqual(config['profile']['plugin'], 'Lucid_Driver::Lucid_Camera')
        self.assertEqual(params['image_encoding'], '')
        self.assertEqual(params['frame_rate'], 15.0)
        self.assertIsInstance(params['frame_rate'], float)
        self.assertIsInstance(params['serial_number'], str)
        self.assertEqual(params['stream_packet_size'], 1400)
        self.assertFalse(params['stream_auto_negotiate_packet_size'])

    def test_serials_and_empty_native_format_survive_ros_parameter_evaluation(self):
        for kind, serial_key in [('lucid', 'serial_number'), ('realsense', 'serial_no')]:
            launch_context = context(kind, serial_number='220903154')
            config = CAMERA._configuration(launch_context)
            evaluated = evaluate_parameters(launch_context, normalize_parameters(config['parameters']))[0]
            self.assertEqual(evaluated[serial_key], '220903154')
            self.assertIsInstance(evaluated[serial_key], str)
            if kind == 'lucid':
                self.assertEqual(evaluated['image_encoding'], '')

    def test_camera_namespaces_keep_source_and_encoder_topics_aligned(self):
        for kind in CAMERA.PROFILES:
            with self.subTest(kind=kind):
                config = CAMERA._configuration(context(kind, camera_namespace='/rig/front'))
                self.assertEqual(config['raw'], '/rig/front/image_raw')
                params = config['encoder_parameters']
                self.assertEqual(params['qos_overrides./rig/front/image_raw.subscription.depth'], 1)
                self.assertEqual(params['qos_overrides./rig/front/image_raw.subscription.reliability'], 'best_effort')
                self.assertEqual(params['out_transport'], 'rtsp')
        root = CAMERA._configuration(context('realsense', camera_namespace='/'))
        self.assertEqual(root['raw'], '/image_raw')

    def test_driver_yaml_is_last_to_allow_application_overrides(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'driver.yaml'
            path.write_text('/**:\n  ros__parameters:\n    enable_depth: true\n')
            config = CAMERA._configuration(context('realsense', driver_params_file=str(path)))
            self.assertEqual(config['parameters'][-1], str(path))
        with self.assertRaisesRegex(ValueError, 'driver_params_file does not exist'):
            CAMERA._configuration(context('realsense', driver_params_file='/not/a/driver/file.yaml'))

    def test_invalid_stream_settings_fail_before_driver_start(self):
        for override in ({'framerate': '0'}, {'bitrate': '0'}, {'rtsp_port': '65536'},
                         {'composed': 'maybe'}, {'max_latency': '-1'}, {'timestamp_source': '3'}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                CAMERA._configuration(context('realsense', **override))
        with self.assertRaisesRegex(ValueError, 'Lucid ROI'):
            CAMERA._configuration(context('lucid', width='640', height='480'))

    def test_optional_driver_is_not_needed_for_argument_introspection(self):
        with patch.object(CAMERA, 'get_package_share_directory', side_effect=PackageNotFoundError('missing')):
            names = {entity.name for entity in CAMERA.generate_launch_description().entities
                     if isinstance(entity, DeclareLaunchArgument)}
        self.assertTrue({'camera_type', 'serial_number', 'driver_params_file', 'composed'} <= names)

    def test_missing_driver_fails_with_optional_dependency_message(self):
        with patch.object(CAMERA, 'get_package_share_directory', side_effect=PackageNotFoundError('missing')):
            with self.assertRaisesRegex(RuntimeError, 'optional lucid_camera_driver'):
                CAMERA._pipeline(context('lucid'))

    def test_composed_and_separate_launch_actions_need_no_vendor_import(self):
        with patch.object(CAMERA, 'get_package_share_directory', return_value=str(ROOT)):
            for kind in ('realsense', 'lucid'):
                with self.subTest(kind=kind):
                    composed = CAMERA._pipeline(context(kind))
                    self.assertTrue(any(isinstance(action, ComposableNodeContainer) for action in composed))
                    self.assertTrue(any(isinstance(action, GroupAction) for action in composed))
                    separate = CAMERA._pipeline(context(kind, composed='false', decode='false'))
                    self.assertEqual(sum(isinstance(action, Node) for action in separate), 2)

    def test_decoder_controls_are_forwarded_to_the_receiving_launch(self):
        with patch.object(CAMERA, 'get_package_share_directory', return_value=str(ROOT)):
            actions = CAMERA._pipeline(context('realsense', max_latency='0.05',
                                               low_latency='false', timestamp_source='0'))
        group = next(action for action in actions if isinstance(action, GroupAction))
        decoder = next(action for action in group.get_sub_entities()
                       if isinstance(action, IncludeLaunchDescription))
        values = dict(decoder.launch_arguments)
        self.assertEqual(values['max_latency'], '0.05')
        self.assertEqual(values['low_latency'], 'false')
        self.assertEqual(values['timestamp_source'], '0')


if __name__ == '__main__':
    unittest.main()
