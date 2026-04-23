# Triceratops 四足機器人操作手冊

## 系統架構

```
ros2 run joy joy_node      ← 讀取實體搖桿
        ↓ /joy
Joy_controller.py          ← 轉換為速度與模式指令
        ↓ /cmd_vel  /robot_mode
Controller.py              ← 馬達控制、步態、動作
```

---

## 啟動步驟

```bash
# 終端 1：主控制程式
cd triceratops_quadruped_robot/triceratops_base/
python3 Controller.py

# 終端 2：搖桿輸入處理
python3 Joy_controller.py

# 終端 3：ROS2 搖桿節點
ros2 run joy joy_node
```

---

## 搖桿按鍵對照表

### 單鍵

| 按鍵 | 功能 | 說明 |
|------|------|------|
| START | 啟動步態 | 開始行走 |
| A | 重置姿態 | 回到初始站立位置 |
| Y | 握手 | 停步 → 支撐站姿 → 抬 FL 腿 → 回復 → 重啟步態 |
| X | 支撐站姿 | 移動到握手前的站立姿勢（debug 用） |

### 組合鍵（按住 LB 再按）

| 組合鍵 | 功能 | 說明 |
|--------|------|------|
| LB + A | 頭往左 | head_pan → 1800 |
| LB + B | 頭往右 | head_pan → 2250 |
| LB + Y | 頭回中 | head_pan → 2048 |

### 安全停止

| 組合鍵 | 功能 | 說明 |
|--------|------|------|
| LB + RB + X | 緊急停止 | 三鍵同按，立即停止步態 |

### 類比搖桿（步態行走時有效）

| 搖桿 | 功能 |
|------|------|
| 左搖桿 上/下 | 前進 / 後退 |
| 左搖桿 左/右 | 左平移 / 右平移 |
| 右搖桿 左/右 | 左轉 / 右轉 |

---

## CLI 指令（直接在 Controller.py 終端輸入）

| 指令 | 功能 |
|------|------|
| `s` | 啟動步態 |
| `stop` | 停止步態 |
| `reset` | 重置姿態 |
| `sit` | 移到支撐站姿 |
| `handshake` | 握手動作 |
| `sway` | 腰部左右晃動 |
| `shake_head_left` | 頭往左 |
| `shake_head_right` | 頭往右 |
| `shake_head_center` | 頭回中 |
| `enable` | 啟用所有馬達 |
| `disable` | 禁用所有馬達 |
| `read` | 顯示最後送出的馬達位置 |
| `exit` | 結束程式 |

---

## 馬達 ID 對照

| ID | 名稱 | 說明 |
|----|------|------|
| 1–3 | FR_higher / lower / hip | 右前腿 |
| 4–6 | FL_higher / lower / hip | 左前腿 |
| 7–9 | RR_higher / lower / hip | 右後腿 |
| 10–12 | RL_higher / lower / hip | 左後腿 |
| 13 | waist_axis1 | 腰部軸 1 |
| 14 | waist_axis2 | 腰部軸 2 |
| 15 | head_tilt | 頭部上下 |
| 16 | head_pan | 頭部左右（center=2048, left=1800, right=2250） |
