# Project Notes

- 已完成发射调试，等待下一步完善项目。

# User Collaboration Preference

- When reviewing or scanning this project, first discuss the findings with the user or explain exactly how the code should be changed. Do not modify project source code unless the user explicitly authorizes the modification.

# Project Knowledge Base

Use the following local files as the long-term knowledge base for this STM32F103RCT6 CubeIDE robot project. Future chats working in this project should consult these files when deciding architecture, pin usage, communication behavior, and task flow.

## Project Workspace

- STM32CubeIDE project path: `D:\stmideexamole\stmide\Robot_rct6_all`

## Competition And Workflow Documents

- Competition task document: `D:\预备队项目\26新生项目比赛题目V2.1.docx`
- Communication protocol document: `D:\预备队项目\通信协议\通信协议.docx`
- Vehicle workflow document: `D:\预备队项目\小车工作流程.docx`
- Basic hardware configuration document: `D:\预备队项目\配置基本.docx`

## Pin And Hardware Reference

- Pin assignment spreadsheet: `D:\预备队项目\引脚分配.xlsx`

## Field / Scene Reference

- Tinkercad field scene image: `D:\HuaweiMoveData\Users\陈思危\Documents\Tencent Files\766298194\nt_qq\nt_data\Pic\2026-05\Ori\4d4814c026bc33627f2a3bb794a0d301.jpg`

## Current Architecture Notes

- The project is for STM32F103RCT6 in STM32CubeIDE.
- Electrical control responsibilities include USB CDC communication with vision/computer side, chassis control, shooter control, servo aiming/feed control, and MPU6500 attitude sensing. The GY-53 distance module is no longer used; legacy USART3/DMA configuration may remain until the next CubeMX cleanup.
- The shooter physical debugging has been completed; the next project phase is to improve the overall robot state machine and connect `robot` with protocol, shooter, servo, and eventually chassis/navigation.
- Keep generated `Debug/` build artifacts out of git unless explicitly requested.
