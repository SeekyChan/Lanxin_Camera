# qr_code_identify_lanxin 开发与维护文档

本文档用于帮助后续维护者理解 `qr_code_identify_lanxin` 新实现。它不是简单的使用说明，而是面向“接手技术债”的开发文档：讲清楚这个包为什么存在、对外兼容什么、内部如何分层、SDK 数据如何进入 ROS、参数如何读写、哪里有并发边界、后续要扩展时应从哪里下手。

当前包所在路径：

```text
driver/qr_code_identify_cbc/qr_code_identify_lanxin
```

外层 `qr_code_identify_cbc` 只是隔离目录。真正的 ROS 包名仍然是：

```text
qr_code_identify_lanxin
```

## 1. 背景与目标

旧版 `driver/qr_code_identify_lanxin` 基本是从厂商 SDK demo 演进出来的 ROS 节点。它能工作，但存在几个典型问题：

- SDK 逻辑、ROS 逻辑、参数读写、重启逻辑混在一起。
- 业务状态大量放在头文件全局变量里。
- SDK 回调线程、ROS service 线程、主循环之间通过多个 atomic 和普通变量共享状态，状态一致性难保证。
- 参数服务使用 `system("rosparam dump/load ...")`，副作用大，不易控制写入内容。
- 相机 handle 生命周期没有被一个明确对象托管，失败路径和重启路径不够清晰。

新实现的目标：

- 保持旧 ROS 接口兼容。
- 把 SDK 生命周期、数据缓存、参数读写、ROS 发布拆开。
- 让 SDK 回调线程只做轻量工作，不直接发布 ROS 消息，不直接改 ROS 参数。
- 用一个线程安全的结果缓存保存“最新一帧完整检测结果”，避免多个字段各自 atomic 导致的半帧数据问题。
- 参数服务更新时既更新当前运行参数，也安全写回 `config_cameras.yaml`，兼容外部调参工具。
- 不把 wrapper 内部参数写进 yaml，保持旧配置 schema 干净。

## 2. 对外兼容接口

### ROS 包与节点

```text
package: qr_code_identify_lanxin
launch : one_qr_camera.launch
node   : qr_code_identify
type   : qr_code_identify_node
```

启动命令：

```bash
roslaunch qr_code_identify_lanxin one_qr_camera.launch
```

### 发布 topic

```text
topic: qr_data
type : common_msgs/qr_data
```

消息结构来自 `common_msgs`：

```text
common_msgs/qr_data
  common_msgs/one_qr_data[] qr_datas

common_msgs/one_qr_data
  int8 type_qr       # 1 华睿, 2 蓝芯
  int8 id_qr         # 1 下视, 2 上视
  int8 valid_qr      # 1 有效, 0 无效
  uint32 value_qr    # 二维码 ID
  float32 x_offset
  float32 y_offset
  float32 theta_offset
```

当前实现默认发布两个元素：

```text
qr_datas[0]：下视相机真实结果，id_qr = 1
qr_datas[1]：上视占位结果，valid_qr = 0，id_qr = 2
```

上视占位是为了兼容旧消费者。当前新实现只接入下视相机。

### 参数服务

```text
service: update_qr_code_identify_lanxin_params
type   : common_msgs/SetLxCameraParams
```

服务定义：

```text
bool flag_update
common_msgs/config_lx_camera new_lx_param
---
common_msgs/config_lx_camera current_lx_param
bool success
string message
```

当前语义：

```text
flag_update = false
  返回当前节点内存中的相机参数，不重启相机。

flag_update = true
  校验 new_lx_param。
  写回 config/config_cameras.yaml。
  写入 ROS 参数服务器。
  更新节点内存配置。
  重启 SDK 相机。
  返回当前参数。
```

与旧实现的唯一明显语义差异：

```text
旧实现 flag_update=false 会重新 load yaml，并触发重启。
新实现 flag_update=false 只读当前参数，不重启。
```

这个变化是有意的：读操作不应带隐藏副作用。如果某个外部工具依赖“读服务也触发重启”，需要单独确认后再兼容。

## 3. 文件与模块结构

```text
qr_code_identify_lanxin/
  CMakeLists.txt
  package.xml
  README.md
  docs/
    DEVELOPMENT.md
  config/
    config_cameras.yaml
  launch/
    one_qr_camera.launch
  include/
    lanxin_sdk_client.h
    qr_camera_config.h
    qr_code_identify_node.h
    qr_code_parser.h
    qr_result_buffer.h
  src/
    lanxin_sdk_client.cpp
    main.cpp
    qr_camera_config.cpp
    qr_code_identify_node.cpp
    qr_code_parser.cpp
    qr_result_buffer.cpp
  sdk/
    include/
      VmrQrCameraApi.h
      VmrQrCameraDefine.h
    lib/
      arm64/
      x86/
```

### 模块职责总览

```text
QrCodeIdentifyNode
  ROS 层。负责 publisher、service、timer、自动重连调度。

LanxinSdkClient
  SDK 适配层。负责 VMR_QCSDK handle 生命周期和 SDK callback 注册。

QrResultBuffer
  跨线程结果缓存。保存最新一帧完整识别结果。

qr_code_parser
  业务解析层。负责 raw code -> uint32 ID，SDK theta -> ROS theta_offset。

qr_camera_config
  参数层。负责默认参数、ROS 参数读取/写入、参数校验、yaml 持久化。
```

## 4. 整体数据流

```text
Lanxin camera hardware
  -> VMR_QCSDK continuous capture
  -> SDK image callback
       只统计 image_count
  -> SDK detection callback
       读取 result->code / x / y / theta / frame / timestamp
       解析二维码 ID
       转换坐标和角度
       写入 QrResultBuffer
  -> ROS timer at publish_rate_hz
       从 QrResultBuffer 读取最新结果
       判断 stale_timeout_sec
       组装 common_msgs/qr_data
       publish 到 qr_data
```

SDK 回调线程不做以下事情：

- 不发布 ROS topic。
- 不调用 service。
- 不读写 ROS 参数服务器。
- 不操作相机 handle。
- 不写 yaml 文件。

这样可以让 SDK 回调保持短小，减少阻塞 SDK 内部采集线程的风险。

## 5. SDK 输入与 ROS 输出映射

SDK 检测回调函数签名：

```cpp
void OnDetection(VmrQrCameraDetectionData* result, void* user_data)
```

厂商 SDK 输出结构：

```cpp
typedef struct {
  char code[64];
  float x;
  float y;
  float theta;
  unsigned int frame_count;
  uint64_t timestamp_ns;
} VmrQrCameraDetectionData;
```

当前转换规则：

```text
raw_code      = result->code
x_offset      = result->x
y_offset      = -result->y
theta_offset  = degrees(normalize(pi / 2 - result->theta))
frame_count   = result->frame_count
sdk_timestamp = result->timestamp_ns
received_time = ros::Time::now()
```

注意：

- SDK 的 `theta` 是弧度。
- 发布出去的 `theta_offset` 目前沿用旧逻辑，是角度。
- `y_offset = -result->y` 也是沿用旧逻辑。

## 6. 二维码 ID 解析规则

代码位置：

```text
src/qr_code_parser.cpp
```

入口：

```cpp
ParsedQrCode ParseQrCode(const std::string& raw_code, int encode_type);
```

当前支持：

```text
encode_type = 1
  蓝芯码。
  要求 raw_code 长度至少 4。
  取 raw_code.substr(1, 3) 作为 ID 字符串。

encode_type = 5
  倍加福码。
  要求 raw_code 长度等于 10。
  取最后 8 位作为 ID 字符串。
```

ID 字符串必须全是数字，并且能放进 `uint32_t`。

解析失败时：

```text
QrDetection.valid = false
QrDetection.value = 0
decode_failure_count + 1
```

当前发布逻辑中，只要最新结果不存在、超时、或者 valid=false，就发布 invalid 数据。

## 7. 参数配置

### yaml 文件

配置文件：

```text
config/config_cameras.yaml
```

该文件故意只保留旧 schema，也就是 `common_msgs/config_lx_camera` 对应字段。不要把 wrapper 内部参数写进去。

保留旧 schema 的原因：

- 兼容旧外部调参工具。
- 兼容旧 `rosparam load` 行为。
- 避免 service 写回 yaml 时把内部运行参数混进去。

### wrapper 内部默认参数

这些参数在代码中有默认值：

```text
qr_type                 = 2
camera_id               = 1
placeholder_camera_id   = 2
publish_placeholder_up  = true
publish_rate_hz         = 100.0
stale_timeout_sec       = 0.06
auto_reconnect          = true
reconnect_interval_sec  = 3.0
update_service_name     = update_qr_code_identify_lanxin_params
```

虽然 `ReadRuntimeOptions()` 支持从 ROS 参数服务器读取它们，但默认 yaml 不写这些字段。这样旧 schema 被外部工具重写后，节点仍能依赖代码默认值正常运行。

如果未来确实需要开放这些参数，建议新建单独文件，例如：

```text
config/wrapper_runtime.yaml
```

不要混入 `config_cameras.yaml`。

## 8. 参数更新与持久化流程

代码位置：

```text
src/qr_code_identify_node.cpp
src/qr_camera_config.cpp
```

`flag_update=true` 时流程：

```text
UpdateParamsCallback()
  -> ValidateLxCameraParams(request.new_lx_param)
  -> WriteLxCameraParamsToDefaultFile(request.new_lx_param)
  -> WriteLxCameraParamsToServer(private_nh_, request.new_lx_param)
  -> config_.lx = request.new_lx_param
  -> RestartCamera("params updated by service")
  -> response.success = true/false
```

### 写回 yaml 的安全策略

写回函数：

```cpp
bool WriteLxCameraParamsToDefaultFile(
    const common_msgs::config_lx_camera& params,
    std::string* error);
```

目标路径通过 `ros::package::getPath("qr_code_identify_lanxin")` 获得：

```text
$(rospack find qr_code_identify_lanxin)/config/config_cameras.yaml
```

写入方式：

```text
1. 写 config_cameras.yaml.tmp
2. flush 并检查 out.good()
3. std::rename(tmp, config_cameras.yaml)
```

这样比直接覆盖原文件安全：如果写临时文件过程中失败，原 yaml 还在。

注意：

- 当前没有保留原 yaml 注释。
- 当前会按固定字段顺序重新生成 yaml。
- 当前不使用 `rosparam dump`，避免把 wrapper 参数或无关参数写入文件。

## 9. 相机启动与重启流程

代码位置：

```text
src/lanxin_sdk_client.cpp
```

启动流程：

```text
StartLocked()
  -> VMR_QC_GetVersion()
  -> VMR_QC_CreateHandle()
  -> VMR_QC_OpenDevice()
  -> VMR_QC_SetDeviceParam(Exposure)
  -> VMR_QC_SetDeviceParam(Gain)
  -> VMR_QC_SetDeviceParam(LedBrightness)
  -> VMR_QC_SetDetectionParams()
  -> VMR_QC_SetImageCallback()
  -> VMR_QC_SetDetectionCallback()
  -> accepting_callbacks = true
  -> VMR_QC_StartCapture()
  -> running_ = true
```

停止流程：

```text
StopLocked()
  -> accepting_callbacks = false
  -> session_id++
  -> if running_: VMR_QC_StopCapture()
  -> VMR_QC_CloseDevice()
  -> VMR_QC_ReleaseHandle()
  -> handle_ = null
  -> running_ = false
  -> result_buffer.Clear()
```

重启流程：

```text
Restart()
  -> StopLocked()
  -> StartLocked(config)
```

## 10. 并发模型

这是新实现最重要的设计点。

### 线程来源

当前主要有两类线程：

```text
ROS spin 线程
  service callback
  publish timer callback
  reconnect timer callback
  status timer callback

SDK 内部回调线程
  image callback
  detection callback
```

当前使用 `ros::spin()`，没有使用 `ros::AsyncSpinner`。因此 ROS 自己的 service/timer 回调在默认情况下串行执行。

### 锁与共享数据

#### LanxinSdkClient::lifecycle_mutex_

保护：

```text
handle_
running_
SDK start/stop/restart 生命周期
```

这个锁只在 ROS 线程中直接使用。SDK 回调不拿这个锁。

#### QrResultBuffer::mutex_

保护：

```text
latest_
has_detection_
```

SDK detection callback 写入完整 `QrDetection`。ROS publish/status timer 读取完整 `QrDetection`。

这避免了旧代码中 `valid/value/x/y/theta` 分别 atomic 导致的半帧数据问题。

#### CallbackContext atomic 字段

```text
accepting_callbacks
encode_type
image_count
detection_count
null_detection_count
decode_failure_count
session_id
```

用途：

- `accepting_callbacks`：停止/重启期间快速拒绝 SDK 回调。
- `session_id`：防止极端情况下旧 session 回调在 stop/restart 后写入旧结果。
- 统计计数使用 atomic，避免在 SDK 回调里加锁。

### 为什么 SDK 回调不直接发布 ROS topic

SDK 回调通常在 SDK 内部采集或识别线程中执行。回调里做太多事情可能阻塞取流或识别。因此当前策略是：

```text
SDK callback:
  解析和写缓存

ROS timer:
  发布 ROS 消息
```

## 11. stale_timeout_sec 的作用

旧代码里如果超过 60ms 没有新识别，就把 `valid` 清零。

新实现不主动修改缓存里的 `valid`，而是在发布时判断结果年龄：

```cpp
result_buffer_.Latest(config_.runtime.stale_timeout_sec, &detection)
```

如果最新结果超过 `stale_timeout_sec`，视为没有有效结果，发布 invalid 数据。

这样缓存仍保留最后一次检测内容，但对外发布不会误报有效。

## 12. 自动重连

启动失败时，节点不会退出。只要配置合法，`Start()` 会尝试启动相机；如果失败，会通过 reconnect timer 周期性重试。

默认：

```text
auto_reconnect = true
reconnect_interval_sec = 3.0
```

重连逻辑：

```text
ReconnectTimerCallback()
  if auto_reconnect && !sdk_client_.IsRunning():
    StartCamera("auto reconnect")
```

注意：

- 只有 SDK client 认为未运行时才重连。
- 运行中掉线后 SDK 是否能反馈 running=false，取决于 SDK 行为。当前没有单独的 SDK 断线事件处理。

未来如果 SDK 有异常停止回调或错误码机制，应在 `LanxinSdkClient` 内部接入。

## 13. 构建依赖

`package.xml` 依赖：

```text
catkin
roscpp
roslib
std_msgs
common_msgs
```

`roslib` 用于：

```cpp
ros::package::getPath("qr_code_identify_lanxin")
```

SDK 库来自：

```text
sdk/lib/arm64/libVMR_QCSDK.so
sdk/lib/x86/libVMR_QCSDK.so
```

CMake 根据 `CMAKE_SYSTEM_PROCESSOR` 选择：

```text
aarch64 / ARM64 / arm64 -> arm64
x86_64 / AMD64 / amd64 -> x86
```

## 14. 运行前检查清单

1. 当前工作区只编译一个 `qr_code_identify_lanxin` 包，避免和旧包同名冲突。
2. `rospack find qr_code_identify_lanxin` 指向新包目录。
3. `config/config_cameras.yaml` 中 `down_qr_camera_ip` 正确。
4. 设备网络可达，例如：

```bash
ping 192.168.100.6
```

5. SDK so 能被加载。CMake 已设置 build/install rpath，但部署环境仍需确认依赖 so 存在。
6. 如果是网口相机，确认系统网络 buffer 和 CPU governor 是否满足现场要求。

## 15. 常见问题

### 15.1 service 更新成功，但重启后参数丢失

检查节点是否有权限写：

```text
$(rospack find qr_code_identify_lanxin)/config/config_cameras.yaml
```

新实现会先写临时文件：

```text
config_cameras.yaml.tmp
```

如果目录不可写，service 会返回：

```text
write config_cameras.yaml failed: ...
```

### 15.2 topic 一直 valid=0

可能原因：

- 相机未启动成功。
- 没有识别到码。
- 识别到了码，但 code 解析失败。
- 最新结果超过 `stale_timeout_sec`。
- `encode_type` 配错。

查看每秒状态日志：

```text
lanxin_qr running=... image_fps=... detection_fps=... decode_fail=...
```

如果 `image_fps > 0` 但 `detection_fps = 0`，说明有图像但没有检测结果。

如果 `detection_fps > 0` 但 `decode_fail > 0`，说明 SDK 有结果，但字符串解析不符合当前规则。

### 15.3 倍加福码解析失败

当前 `encode_type=5` 要求 raw code 长度等于 10，并取最后 8 位。

如果现场码格式和这个规则不一致，要改：

```text
src/qr_code_parser.cpp
```

### 15.4 theta 单位看起来不对

当前发布的 `theta_offset` 是角度，不是弧度。这是为了兼容旧实现。

转换规则：

```text
theta_offset = degrees(normalize(pi / 2 - sdk_theta))
```

如果后续想改成弧度，必须同步检查所有消费 `qr_data` 的下游节点。

### 15.5 更新参数后相机短暂无数据

这是正常的。参数更新会执行 SDK restart：

```text
StopCapture -> CloseDevice -> ReleaseHandle -> Create/Open/Config/Start
```

重启期间 `QrResultBuffer` 会清空，topic 发布 invalid。

## 16. 后续扩展建议

### 16.1 支持双相机

当前只真正接入下视相机，上视是占位 invalid。

推荐扩展方式：

```text
LanxinSdkClient down_client;
LanxinSdkClient up_client;
QrResultBuffer down_buffer;
QrResultBuffer up_buffer;
```

不要把上下相机逻辑塞进同一个 client。每个相机有独立 handle、独立 callback context、独立 result buffer。

ROS 发布时：

```text
qr_datas[0] = down_buffer latest
qr_datas[1] = up_buffer latest
```

### 16.2 增加图像发布

当前 image callback 只统计帧数。如果未来需要发布图像：

- 不建议直接在 SDK image callback 中 publish。
- 建议增加 image buffer 或 queue。
- ROS timer 或独立 ROS worker 消费队列后发布 `sensor_msgs/Image`。

### 16.3 参数动态生效而不重启

当前所有参数更新都重启 SDK 相机。若 SDK 支持运行时设置部分参数，可以拆成：

```text
设备参数：曝光、增益、LED，可尝试在线 set。
算法参数：rows/cols/size/encode_type 等，保守起见仍 restart。
IP 参数：必须 restart。
```

但这需要基于 SDK 行为验证，不能只看接口猜。

### 16.4 单元测试

优先给这些纯逻辑补测试：

```text
ParseQrCode()
ConvertSdkThetaToDegrees()
ValidateLxCameraParams()
WriteLxCameraParamsToDefaultFile()
```

SDK client 难做纯单测，可以后续用接口抽象或 mock SDK 函数。

## 17. 维护原则

后续改这个包时，建议遵守：

- 不要在头文件里定义全局业务变量。
- 不要让 SDK callback 直接操作相机生命周期。
- 不要在 SDK callback 中执行阻塞 IO。
- 不要用 `rosparam dump` 写 `config_cameras.yaml`，避免写入无关参数。
- 不要把 wrapper 内部参数混进旧 schema 配置文件。
- 如果要新增 ROS 接口，先确认旧下游是否依赖数组长度、topic 名、service 名。
- 如果要改角度单位或坐标符号，必须先查下游定位融合逻辑。

## 18. 快速定位代码入口

```text
main()
  src/main.cpp

ROS node 类
  include/qr_code_identify_node.h
  src/qr_code_identify_node.cpp

SDK client
  include/lanxin_sdk_client.h
  src/lanxin_sdk_client.cpp

最新结果缓存
  include/qr_result_buffer.h
  src/qr_result_buffer.cpp

参数与 yaml 写回
  include/qr_camera_config.h
  src/qr_camera_config.cpp

二维码字符串和角度转换
  include/qr_code_parser.h
  src/qr_code_parser.cpp
```

## 19. 当前已知取舍

- 没有保留旧 yaml 注释，写回时按固定格式重新生成。
- `flag_update=false` 不重启相机。
- wrapper runtime 参数暂时不写入 yaml。
- 相机运行中异常断线后的状态感知依赖 SDK 行为，目前只有定时重连未运行状态。
- 上视相机仍为占位输出，不是真实 SDK client。

这些不是遗漏，而是当前阶段为了清晰、兼容和降低并发复杂度做出的取舍。
