# nc1020-cardputer-fast

文曲星 NC1020 模拟器，M5Cardputer (ESP32-S3, 8MB flash, 无 PSRAM)，ESP-IDF。
这是 `cardputer-nc1020/`（SD 分页版，作为存档保留）的**提速分支**：把 ROM 和 NOR
搬进片上 flash，词典直读、秒启、几乎不依赖 SD。

## 相比存档版做了什么

| 改动 | 效果 |
|---|---|
| **ROM 前 6.44MB（206 bank）进 flash** | 首次开机从 SD 拷入 `romdata` 分区并 `esp_partition_mmap` 成直接指针 → 词典零换页、cache 速度直读。尾部 162 bank 仍走 SD。 |
| **NOR（512KB 用户数据）进 flash** | `nordata` 分区，首次从 SD `.nor` 播种；页读自 flash、写到 RAM 槽、脏页 erase+write 回 flash。数据持久、不依赖 SD `.nor`。 |
| **CPU 寄存器缓存** | `CpuExecuteOP` 把 mPC/mA/mX/mY/mSP 等缓存进局部变量（避开 char 别名导致的反复重载），并改成单函数批处理循环。 |
| **240MHz + QIO flash + 去掉每帧空转 vTaskDelay + 仅变化重绘** | 综合提速。 |
| **全屏显示 + 跳过第0列** | 160×80→240×135 全屏（180° 旋转）。第0列是状态图标条数据（非像素），跳过避免边缘杂点线。 |
| **电脑式键盘 + Fn 功能层** | 顶排=数字、字母 QWERTY、回车/空格/方向键。物理 **Fn 键 + 数字 = F1~F11**（F1-4 编辑键；F5=英汉 F7=计算 F9=资料 …）。 |

## flash 分区（partitions.csv，8MB 全用满）

```
nvs       0x009000  0x06000   # 含 romsig/norinit 填充标记
phy_init  0x00f000  0x01000
factory   0x010000  0x100000  # 1MB app（实际 ~0.6MB）
romdata   0x110000  0x670000  # 6.44MB = 206 个 32KB ROM bank（mmap 只读）
nordata   0x780000  0x080000  # 512KB NOR（读写，用户数据）
```

## 首次开机

从 SD 把 ROM(206 bank) 拷进 `romdata` + NOR 播种，**约 70~100 秒，带进度条**。
NVS 标记完成后，**以后每次开机秒启**。换了 SD 上的 `.rom`/`.nor` 会因签名变化自动重拷。

强制重载：`idf.py -p COM5 erase-flash` 再 `idf.py -p COM5 flash`（清掉 NVS 标记）。

## 构建 / 烧录

```powershell
& "C:\Espressif\Initialize-Idf.ps1"
idf.py -p COM5 flash monitor      # COM5 换成你的串口
```

SD 卡根目录需有 `nc1020.rom`（必须正好 12MB）和（可选）`nc1020.nor`。

## 速度 / 注意

- 词典在 flash bank 内：直读、不卡。整体仍受 6502 解释器限制（标志位还可进一步缓存提速，但有正确性风险，未做）。
- NOR 写：脏页 erase+write 8KB 回 flash（每 5s flush 一次脏页）。重度"资料整理"会有秒级停顿并消耗 flash 写寿命（~10万次/扇区），日常使用无虞。
- 关键代码：`components/nc2000/flash_rom.{h,cpp}`（ROM mmap + NOR flash 后端）、
  `sd_paging.cpp`（槽缓存，ROM 走 flash 直读 / NOR 走 flash 后端 / 尾部走 SD）、
  `main/main.cpp`（`draw_lcd` 显示、`kb_map`/`extend_kb_map` 键盘、`flash_rom_prepare`/`nor_flash_prepare`）。
- `main.cpp` 仍有调试用串口心跳日志（`heartbeat:`），可去掉出更干净版。
