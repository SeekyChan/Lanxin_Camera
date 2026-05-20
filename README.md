# qr_code_identify_lanxin

蓝芯 / VMR 二维码相机 SDK 的 ROS 封装包。它负责启动 SDK 相机、接收二维码识别结果，并将二维码 ID 与位姿偏差发布到 ROS。

## 运行接口

- 发布 topic：`qr_data`
- topic 类型：`common_msgs/qr_data`
- 参数服务：`update_qr_code_identify_lanxin_params`
- 服务类型：`common_msgs/SetLxCameraParams`
- 启动文件：`launch/one_qr_camera.launch`
- 默认节点名：`qr_code_identify`
- 可执行文件：`qr_code_identify_node`

启动示例：

```bash
roslaunch qr_code_identify_lanxin one_qr_camera.launch
```

## 参数服务语义

`flag_update=false`：

```text
只返回当前节点内存中的相机参数，不重启相机。
```

`flag_update=true`：

```text
校验 new_lx_param
写回 config/config_cameras.yaml
写入 ROS 参数服务器
更新节点内存配置
重启 SDK 相机
```

## 配置文件说明

`config/config_cameras.yaml` 只保留相机参数字段，也就是 `common_msgs/config_lx_camera` 对应字段。

## 数据流

```text
蓝芯相机
  -> VMR_QCSDK 连续采集
  -> SDK 图像回调：只统计帧数
  -> SDK 识别回调：输出 code / x / y / theta
  -> QrResultBuffer：保存一整帧线程安全的最新识别结果
  -> ROS 定时器：按发布频率输出 common_msgs/qr_data
```

## 主要模块

- `LanxinSdkClient`：管理 SDK handle 生命周期，负责 start / stop / restart。
- `QrResultBuffer`：保存最新一帧识别结果，隔离 SDK 回调线程和 ROS 发布线程。
- `QrCodeIdentifyNode`：管理 ROS publisher、service 和 timer。
- `qr_code_parser`：解析 SDK 原始 code 字符串，转换二维码 ID 和角度。
- `qr_camera_config`：读取、校验、写入 ROS 参数，并安全写回 `config_cameras.yaml`。

