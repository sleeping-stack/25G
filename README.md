# 25G — 电路模型探究装置

2025 全国大学生电子设计竞赛 G 题解决方案，基于 STM32H743VIT6（Cortex-M7，480MHz，硬浮点）。

## 目录结构

```
25G/
├── Core/
│   ├── Inc/                    # CubeMX 生成头文件（不可改，USER CODE 块除外）
│   ├── Src/                    # CubeMX 生成源文件（main/adc/dac/dma/tim/usart/gpio）
│   └── User/                   # 用户自有代码（CubeMX 永不覆盖）
│       ├── Inc/
│       │   ├── adc_sampling.h  # ADC 双通道采样接口
│       │   ├── dds_interface.h # DDS 抽象接口（AD9959）
│       │   ├── AD9959.h        # AD9959 驱动
│       │   ├── iir_filter.h    # 2 阶 IIR biquad
│       │   ├── dac_output.h    # DAC1 实时输出
│       │   ├── sysid.h         # 系统辨识（建模）
│       │   ├── ui.h            # 串口屏显示后端
│       │   └── app_state.h     # 应用状态机
│       └── Src/                # 对应 .c 实现
├── Drivers/                    # HAL + CMSIS + CMSIS-DSP（vendored）
├── cmake/                      # 工具链与 CubeMX 生成 CMake
├── 25G.ioc                     # CubeMX 工程文件
├── STM32H743XX_FLASH.ld        # 链接脚本
├── CMakeLists.txt              # 根构建
├── CMakePresets.json            # Debug/Release 预设
```

## 构建

```bash
cmake --preset Debug          # 配置（首次或新增文件后）
cmake --build build/Debug     # 编译 → build/Debug/25G.elf
```

工具链 `cmake/gcc-arm-none-eabi.cmake`，需 `arm-none-eabi-gcc` 在 PATH。

## 外设分配

| 外设 | 引脚 | 用途 |
|---|---|---|
| ADC1 + ADC2 | PC1, PC0 | dual simultaneous 采样，建模取 X/Y |
| DAC1 | PA4 | 实时 IIR 输出 |
| TIM6 | — | 触发源，Period=292，fs≈819.2kHz |
| USART1 | PA9/PA10 | 陶晶驰串口屏 |
| AD9959 | GPIO SPI | CH3 扫频激励 |