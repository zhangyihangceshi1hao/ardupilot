// ============================================================================
//  AP_Choreo_config.h
// ----------------------------------------------------------------------------
//  AP_Choreo（无人机表演舞步执行库）的编译开关头文件。
//
//  整个 AP_Choreo 库 + ArduCopter 集成点全部由 `AP_CHOREO_ENABLED` 宏控制。
//  关掉（=0）时，所有 .cpp 文件主体都被 #if 块跳过，等效于该库不存在 —
//  ArduCopter 行为跟原版 ArduPilot 完全一致，零侵入。
//
//  目前默认 1（开发期 SITL/调试），方便不加 waf 选项也能编进固件。
//  正式生产用，要走 `--enable-choreo` waf 选项或者通过 hwdef 覆盖：
//        ./waf configure --board <YourBoard> --extra-hwdef=disable_choreo.dat
//  其中 disable_choreo.dat 写一行：  undef AP_CHOREO_ENABLED \ define AP_CHOREO_ENABLED 0
//
//  作者：FormationPerformance / Zhang-Yi-Hang  2026-05
//  参考文档：D:\Study_Code\模型\docs\FIRMWARE_PLAN.md
// ============================================================================
#pragma once

#ifndef AP_CHOREO_ENABLED
// Phase 6.6：默认 ON 方便 SITL 开发期编进固件。
// 生产构建可通过 hwdef 或环境变量覆盖成 0。
#define AP_CHOREO_ENABLED 1
#endif
