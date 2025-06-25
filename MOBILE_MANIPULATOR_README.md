# 移動操作平台模擬系統

## 概述
此專案已成功從Spot四足機器人轉換為輪式移動操作平台，適用於ROS2 Jazzy。

## 已完成的主要修改

### 1. 系統架構升級
- ✅ 確認ROS2 Jazzy兼容性
- ✅ 保留核心Webots模擬環境
- ✅ 維持MoveIt機械手臂控制

### 2. 新增檔案
- `resource/mobile_manipulator.urdf` - 完整的輪式移動操作平台URDF模型
- `webots_spot/mobile_driver.py` - 差速驅動控制器
- `resource/mobile_manipulator_controllers.yaml` - ROS2控制器配置
- `launch/mobile_manipulator_launch.py` - 完整系統啟動檔案

### 3. 機器人規格
**移動底盤:**
- 差速驅動系統
- 輪距: 0.6m
- 輪半徑: 0.1m
- 支援/cmd_vel控制

**機械手臂:**
- 6DOF關節型手臂
- 2DOF平行夾爪
- MoveIt運動規劃支援
- ROS2 Control架構

**感測器:**
- IMU慣性測量單元
- 雷射雷達
- 相機
- 輪式編碼器

## 啟動系統

### 基本啟動
```bash
# 建置專案
colcon build --packages-select webots_spot

# 啟動環境
source install/setup.bash

# 啟動移動操作平台 (需要先創建Webots world檔案)
ros2 launch webots_spot mobile_manipulator_launch.py
```

### 遙控操作
```bash
# 鍵盤控制移動底盤
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 機械手臂控制
```bash
# 使用MoveIt進行軌跡規劃
ros2 launch webots_spot moveit_launch.py
```

## 下一步開發建議

### 必要步驟
1. **創建Webots世界檔案** - 需要創建 `worlds/mobile_manipulator.wbt`
2. **配置RViz** - 創建適合的RViz配置檔案
3. **調試控制器** - 微調PID參數和運動學參數

### 擴展功能
1. **導航系統** - 整合Nav2導航堆疊
2. **SLAM功能** - 加入同步定位與建圖
3. **視覺處理** - 整合物體檢測和識別
4. **抓取規劃** - 開發自動抓取策略

### 可選改進
1. **感測器融合** - 整合多感測器數據
2. **任務規劃** - 高階任務執行框架
3. **安全機制** - 碰撞檢測和避障
4. **人機介面** - GUI控制面板

## 測試狀態
- ✅ 套件建置成功
- ✅ 模組導入測試通過
- ⏳ Webots模擬測試 (需要world檔案)
- ⏳ 運動控制測試 (需要完整環境)

## 架構優勢
1. **模組化設計** - 移動底盤和手臂分別控制
2. **標準接口** - 使用ROS2標準msg/srv
3. **擴展性佳** - 易於添加新功能
4. **兼容性好** - 支援最新ROS2 Jazzy

此移動操作平台系統為您的機器人應用提供了堅實的基礎架構！