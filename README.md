# cardputer-nc1020

文曲星 **NC1020** 模拟器，跑在 **M5Cardputer**（ESP32-S3 / StampS3，8MB flash，**无 PSRAM**），ESP-IDF。

把 ROM 和 NOR 搬进片上 flash（`esp_partition_mmap` 直读），词典秒启、几乎不依赖 SD 速度；
支持开机选 ROM、状态存档/恢复、电脑式键盘。wqx 6502 核心来自
[github.com/wangyu-/NC2000](https://github.com/wangyu-/NC2000)。

---

## 功能

- **ROM/NOR 进片上 flash**：词典直读零换页，开机即用；尾部 ROM bank 仍可 SD 分页兜底。
- **状态存档 / 恢复**：自动存档（屏幕静止、进待机、G0 手动），下次开机从上次会话继续，不再每次"资料整理"。
- **开机选 ROM 菜单**：开机按住 **G0** 弹出菜单，扫描 SD `/sd` 与 `/sd/roms` 下的 12MB `.rom`。
- **电脑式键盘**：顶排数字、字母 QWERTY、回车/空格/方向键；**Fn+数字 = NC1020 功能键 F1~F11**。
- **全屏显示**：160×80 软件旋转 90° + 最近邻缩放铺满 240×135。

## 按键

| 操作 | 作用 |
|---|---|
| 普通按键 | 文曲星键盘（数字 / 字母 / 回车 / 空格 / 方向） |
| **Fn + 数字** | 功能键 F1~F11（F1-4 插入/删除/查找/修改；F5 英汉 F7 计算 F9 资料 …） |
| **G0**（侧键，运行中） | **手动立即存档** |
| **开机时按住 G0** | 进入选 ROM 菜单（`;`上 `.`下 `Enter`选 `Esc`取消） |
| **Fn + Del** | **软重启**：删存档 + 文曲星固件重启（卡死/坏档时恢复用） |
| **硬件 RST 按钮** | 复位 ESP，**正常载入存档**（恢复上次会话，不删档） |

存档时机：① 按 G0；② 屏幕静止 >4s（在菜单/主界面发呆时，安全点）；③ 进待机（clk-off）。
NOR 用户数据每 ~5s 刷盘，断电也不丢文档。

## flash 分区（partitions.csv，8MB 全用满）

```
nvs       0x009000  0x06000   # 含 romsig/norinit 填充标记 + cur_rom（上次选的 ROM）
phy_init  0x00f000  0x01000
factory   0x010000  0x100000  # 1MB app（实际 ~0.6MB）
romdata   0x110000  0x670000  # 6.44MB = 206 个 32KB ROM bank（mmap 只读）
nordata   0x780000  0x080000  # 512KB NOR（读写，用户数据）
```

## 构建 / 烧录

需要 ESP-IDF v5.5+：

```powershell
& "C:\Espressif\Initialize-Idf.ps1"     # 激活 ESP-IDF 环境
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor             # COM5 换成你的串口
```

仅烧 app（rom/nor/nvs 已在 flash 时）：`idf.py -p COM5 app-flash`。
强制重载 ROM/NOR：`idf.py -p COM5 erase-flash` 后再 `flash`（清掉 NVS 标记）。

## SD 卡

- 把 ROM 放到 SD 根目录或 `/sd/roms/`，文件名 `*.rom`，**必须正好 12MB**；同名 `.nor` 可选。
- 首次开机会从 SD 把 ROM(206 bank) 拷进 `romdata` + NOR 播种（**~70~100 秒，带进度条**），之后秒启。
- 存档写在 SD（`<rom名>.state`），所以**存档/恢复需要 SD 能正常挂载**。

> 本仓库**不含 ROM/NOR/成品镜像**（版权 + 体积）。请自备 NC1020 ROM。

## 关键代码

- `components/nc2000/` — wqx/NC2000 6502 核心（CPU、内存银行、IO、NOR、状态存读）。
- `components/nc2000/flash_rom.{h,cpp}` — ROM `mmap` + NOR flash 后端。
- `components/nc2000/sd_paging.cpp` — 8KB 槽缓存（ROM 走 flash 直读 / NOR 走 flash / 尾部走 SD）。
- `main/cardputer_bsp.c` — ST7789 显示（**原生 135×240，不用 swap_xy，gap 52/40**）、SD（SPI3，20MHz）、键盘矩阵。
- `main/main.cpp` — `draw_lcd`（软件旋转铺满）、`kb_map`/`extend_kb_map`、选 ROM 菜单、存档/恢复、G0/Fn+Del。

## 说明 / 注意

- 显示是软件旋转 + 最近邻缩放铺满，文字略有锯齿（非整数缩放固有，换 1:1 可更清晰但画面偏小）。
- 设备无 RTC 芯片、无网络，时钟绝对时间无来源；自动时间同步已关闭，**在 NC1020 菜单里手动设一次时间**即随存档保留并走动（约 76% 实时）。
- 整体速度仍受 6502 解释器限制；NOR 写会消耗 flash 写寿命（~10 万次/扇区），日常无虞。

## 致谢

- wqx / NC1020 6502 核心：[wangyu-/NC2000](https://github.com/wangyu-/NC2000)
- 参考实现：[M5Cardputer-NC1020-Emu](https://github.com/)（M5GFX 显示参考）

请遵循 wqx 核心上游的开源协议。
