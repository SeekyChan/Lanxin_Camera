# qr_code_identify

Clean ROS wrapper for the Lanxin / VMR QR camera SDK.

Full development and maintenance notes are in:

```text
docs/DEVELOPMENT.md
```

## Runtime Interfaces

- Publishes `common_msgs/qr_data` on `qr_data`.
- Provides `common_msgs/SetLxCameraParams` on
  `update_qr_code_identify_lanxin_params`.

`flag_update=false` returns the current active camera parameters.
`flag_update=true` validates and writes new camera parameters, then restarts the SDK
camera session.

`config/config_cameras.yaml` intentionally keeps the legacy camera parameter
schema only. Wrapper behavior such as publish rate, stale timeout and reconnect
interval uses code defaults, so older tools that rewrite the YAML with only
`common_msgs/config_lx_camera` fields remain compatible.

## Data Flow

```text
Lanxin camera
  -> VMR_QCSDK continuous capture
  -> SDK image callback: frame counter only
  -> SDK detection callback: code / x / y / theta
  -> QrResultBuffer: one thread-safe latest-result snapshot
  -> ROS timer: common_msgs/qr_data at publish_rate_hz
```

The SDK callback thread never publishes ROS messages and never touches ROS
parameters. The ROS thread owns parameter service handling and timer publishing.

## Main Modules

- `LanxinSdkClient`: owns SDK handle lifecycle.
- `QrResultBuffer`: stores the latest detection as one coherent snapshot.
- `QrCodeIdentifyNode`: owns ROS publisher, service and timers.
- `qr_code_parser`: converts raw SDK code text to `uint32` QR IDs.
- `qr_camera_config`: reads, writes and validates ROS parameters.
