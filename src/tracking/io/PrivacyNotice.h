#pragma once
// Notice shown before tracking data is recorded (app: 空间 → 录制; hidtool: capture / trace). A .lhrec file holds
// the raw HID stream, from which the wearer's head, hand and body motion can be reconstructed. Keep README.md's
// "隐私与数据保护" section in sync with this text.

namespace lhp::tracking::io {

inline constexpr const char* kRecordingNoticeTitle = "录制前请阅读：身体追踪数据保护提示";

inline constexpr const char* kRecordingNoticeWhat =
    "录制文件（.lhrec）保存设备的原始HID数据：IMU（加速度、角速度）、基站扫描时刻、按键与手指传感器读数（如有），"
    "以及设备和基站的序列号。由这些数据可以还原出佩戴者在录制期间头部、双手和身体的完整运动轨迹。";

inline constexpr const char* kRecordingNoticeWhy =
    "身体动作数据是需要特别保护的个人信息：\n"
    "·步态、手势、动作习惯等运动特征可用于识别特定个人，也可能被用来推断健康状况、身体残障、年龄或情绪状态。\n"
    "·中国《个人信息保护法》：生物识别、医疗健康、行踪轨迹等信息属于敏感个人信息（第二十八条），处理须取得个人的单独同意"
    "（第二十九条）；向其他个人信息处理者提供须告知接收方情况并取得单独同意（第二十三条）；向境外提供还须满足第三十八、三十九条的要求。\n"
    "·欧盟GDPR：用于识别特定自然人的生物识别数据及健康数据属于特殊类别数据（第九条），原则上禁止处理，除非取得明确同意等例外情形。"
    "其他地区（如美国伊利诺伊州BIPA）也有关于生物识别信息的法规。";

inline constexpr const char* kRecordingNoticeRules =
    "请遵守：\n"
    "1. 只录制你自己的设备和动作。录制他人前，先告知录制内容、用途和保存期限，并取得对方明确同意。\n"
    "2. 不要把录制文件发送、上传或分享给未经授权的第三方，包括公开的问题追踪、论坛、网盘和聊天群。"
    "确需提供给开发者排查问题时，只提供必要的片段，并通过可信渠道直接交给获得授权的人。\n"
    "3. 录制文件只保存在本机，本程序不会上传任何数据。用完请及时删除，不要长期保存不再需要的录制。";

inline constexpr const char* kRecordingNoticeDisclaimer = "本提示不构成法律意见，具体义务以你所在地的法律为准。";

/// One line for tooltips and console output.
inline constexpr const char* kRecordingNoticeShort =
    "录制文件包含可还原身体动作的追踪数据（敏感个人信息），请勿发送给未经授权的第三方；录制他人前须取得其明确同意。";

}  // namespace lhp::tracking::io
