# PID控制优化说明

## 修改目标
让PWM的stepping更激进，修改PID参数减少过冲问题。

## 主要改进

### 1. PID参数调整
- **默认参数修改**：
  - VEL_KP: 300 → 450 (增大比例系数，提高响应速度)
  - VEL_KI: 300 → 180 (减小积分系数，减少过冲)

### 2. PWM步进值优化
- **普通车型**：
  - smooth_MotorStep: 0.05f → 0.15f (3倍提升，更激进)
  - smooth_ServoStep: 20 → 30 (舵机响应更快)

- **高速阿克曼车型(type=9)**：
  - smooth_MotorStep: 0.15f → 0.25f (最激进设置)
  - smooth_ServoStep: 30 → 40 (舵机响应最快)

### 3. 改进的PI控制算法
实现了积分分离控制，在`Incremental_MOTOR`函数中：

- **积分分离**：当偏差>0.3时，积分系数降低到30%，减少过冲
- **输出增量限幅**：限制单次输出变化量不超过FULL_DUTYCYCLE/8，保证稳定性
- **改进的算法**：分别计算比例和积分项，提高控制精度

### 4. 特定车型PID优化
- **高速阿克曼(case 9)**：KP=350, KI=120
- **顶配独立(case 4)**：KP=500, KI=150 (高精度)
- **顶配独立(case 5)**：KP=380, KI=160 (重载优化)

## 技术原理

### 积分分离控制
```c
//积分分离：当偏差较大时减少积分作用，减少过冲
if( fabs(p->Bias) > 0.3f ) {
    integral *= 0.3f; //偏差大时积分系数降低
}
```

### 输出限幅
```c
//输出增量限幅，防止输出变化过大
if( delta_output >  FULL_DUTYCYCLE/8 ) delta_output =  FULL_DUTYCYCLE/8;
if( delta_output < -FULL_DUTYCYCLE/8 ) delta_output = -FULL_DUTYCYCLE/8;
```

## 预期效果
1. **响应速度提升**：更大的步进值和比例系数
2. **减少过冲**：积分分离和降低积分系数
3. **提高稳定性**：输出增量限幅防止震荡
4. **车型优化**：针对不同车型的专门调优

## 修改文件列表
1. `BALANCE/Inc/robot_select_init.h` - VEL_KP/VEL_KI定义
2. `CarType/Inc/robot_init.h` - VEL_KP/VEL_KI定义
3. `BALANCE/balance_task.c` - PI控制算法和步进值
4. `BALANCE/robot_select_init.c` - 各车型PID参数

## 调试建议
如果发现震荡或不稳定，可以：
1. 适当降低smooth_MotorStep值
2. 调整积分分离阈值(当前0.3f)
3. 修改输出增量限幅比例(当前1/8)
