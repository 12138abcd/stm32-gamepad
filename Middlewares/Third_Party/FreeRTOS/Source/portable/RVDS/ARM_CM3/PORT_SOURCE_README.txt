这两个文件是【GCC 端口】的内容，放在 RVDS 路径下，原因：
CubeMX 对 MDK-ARM 只生成 RVDS 端口（ARMCC/AC5 语法 __forceinline / __asm{}），而 MDK 5.37+ 的 ARMCLANG(AC6) 不认 —— 编译 port.c 会报 unknown type name '__forceinline'。
ARMCLANG 兼容 GCC 风格内联汇编，故用 GCC 端口的 port.c + portmacro.h 替换。
来源：STM32Cube_FW_F1_V1.8.7\\Middlewares\\Third_Party\\FreeRTOS\\Source\\portable\\GCC\\ARM_CM3
注意：CubeMX 每次 GENERATE CODE 都会覆盖回来 → 必须跑 tools/fix-generated-code.mjs 自动修复。