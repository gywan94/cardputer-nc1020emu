# cardputer-nc1020 移植说明

把 `tab5-nc2000`(ESP-IDF / Tab5)的 wqx NC1020 模拟器移到 **M5Cardputer**
(ESP32-S3,PlatformIO + Arduino)。**只做 NC1020(`.rom` + `.nor`)。**

> **注:实际工程是 ESP-IDF(`components/nc2000/` + `main/`),不是下文早期设想的
> PlatformIO/Arduino(`lib/wqx`)。下面的 Arduino 细节已过时,以代码为准。**

## ✅ Milestone 3 已完成:ROM/NOR 从 SD 按需分页(无 PSRAM)

无 PSRAM,12MB ROM 和 512KB NOR 都放不进 RAM,改为从 SD 的 `.rom`/`.nor`
**按 32KB bank 分页**。新增 `components/nc2000/sd_paging.{h,cpp}`,用
`-DNC2K_SD_PAGED` 开关(已在组件 CMake 打开):

- **槽位缓存**:6 个 32KB 槽(192KB,内部 RAM)。每个 super_switch 用"代号
  (generation)"把本次切换解析到的 bank 钉住,保证一次切换内不会自相驱逐。
  单次切换最多 3 个活跃 bank(nor[0] + nor[bbs] + 选中 bank),6 槽足够 + 留 LRU。
- **ROM**:只读,`sdpg_rom(volume,bank_idx)` 按 init_rom 的卷映射算文件偏移读入。
- **NOR**:读写,写命中槽→标脏;驱逐/`sdpg_flush()`/退出时回写 `.nor`;整片擦除
  走 `sdpg_nor_mass_erase()`(常驻槽 memset、非常驻直接写文件 0xff)。主循环每 ~5s
  flush 一次脏页(断电不丢数据)。
- 钩子:`mem.cpp` 的 `GetBank`/`SwitchBbsBios_67`/`super_switch`,`nor.cpp` 的
  `in_nor_range`/mass-erase/byte-program/block-erase/`init_nor`/`SaveNor`,
  `rom.cpp` 的 `init_rom`。`RUN_EMULATOR` 已置 1。
- **未做**:160×80→240×135 显示 blit(CopyLcdBuffer 已拿到帧,`main.cpp` 里仍是
  TODO)。先靠串口日志验证 ROM/NOR 分页与按键。
- SD 根目录放 `nc1020.rom`(必须正好 12MB)+ 可选 `nc1020.nor`(缺失则自动建空白)。

## 当前状态:脚手架完成,尚未编译/上机

- ✅ 目录结构 + `platformio.ini`(board `m5stack-stamps3`,Arduino,M5Cardputer 库)
- ✅ wqx 核心源码复制到 `lib/wqx/`(和 Tab5 同一份,含 `iv_uart.cpp` RTC 修复)
- ✅ Arduino stub `src/cardputer_stubs.cpp`(SDL/Dsp/sound/console/cmd/disassembler 空实现;IV 走真实 `iv_uart.cpp`)
- ✅ 胶水 `src/main.cpp`(M5GFX 显示 160×80→缩放、Cardputer QWERTY 键盘→`SetKeyWayback`、SD 找 `.rom`、核心配置 + 主循环)
- ❌ **未编译**(本机无 PlatformIO CLI)— 需要你用 VSCode PlatformIO 插件编译,把错误发回来迭代
- ❌ **SD 按需 ROM/NOR 未实现** — 这是能否跑起来的关键(见下)

## 两个必须解决的阻塞

### 1. 核心在 Arduino/ESP32-S3 下能否编译

wqx 核心原本在 ESP-IDF 下编译(`-std=gnu++17 -fpermissive -w -DHANDYPSP -DTAB5_PORT`)。
PlatformIO 已在 `platformio.ini` 设了同样的 flags。预期可能要修的点:
- `lib/wqx/stub/` 的 SDL shim 路径(`-I lib/wqx/stub` 已加)
- Arduino 的 `String`/`min`/`max` 宏可能和 STL 冲突 → 必要时 `-DNO_GLOBAL_INSTANCES` 或 `#undef`
- `esp_heap_caps.h` / `esp_timer.h` 在 Arduino-ESP32 下可用(底层就是 IDF),应该 OK

**第一步建议:先编译脚手架,让核心在 S3 上过编译。** 把报错贴给我修。

### 2. 无 PSRAM → ROM(12MB)和 NOR(512KB)放不下内存

StampS3 = ESP32-S3FN8:**8MB flash,无 PSRAM,512KB SRAM**(可用 heap ~300KB)。
而 `lib/wqx/rom.cpp` 现在的做法是 `heap_caps_malloc(12MB, MALLOC_CAP_SPIRAM)` 把整个
ROM 放内存 —— 在 Cardputer 上会**分配失败**,`LoadNC2k()` 崩。

必须改成**从 SD 按需读 bank**(rainyx 的 Cardputer 版就是这么做的:`readRomBank`
每次 bank 切换从 SD 读 32KB)。设计如下,用 `#if NC2K_SD_PAGED`(已在 platformio.ini 定义)隔开:

**ROM(只读,简单):**
- 不分配 `rom_buff`。开一个**单/多 bank 缓存**(如 4 个 32KB slot = 128KB,LRU)。
- `mem.cpp` 的 `GetBank(bank_idx)`:nc1020 的 ROM bank(`bank_idx>=0x80`)时,
  从 SD `.rom` 文件读该 bank(偏移 `0x8000*((bank_idx-0x80)+...)`,注意 nc1020 的
  volume 映射 `rom_volume0/1/2`)到缓存 slot,返回 slot 指针。`memmap[2..5]` 指向它。
- ROM bank 切换不频繁,单 bank 缓存即可起步;来回切换卡顿再上 LRU。

**NOR(可写,复杂):**
- nc1020 把 NOR 映射在 `memmap[6,7]`(`SwitchBbsBios_67` 的 `bbs_pages`,来自 `nor_banks[0..3]`)。
- 需要**带写回的 SD 页缓存**:读 NOR 页 → 缓存;`write_nor0` 写 → 写缓存 + 标脏;
  退出/保存时 flush 脏页回 SD `.nor`。资料整理会大量擦/写 NOR,这块要正确。
- 备选:若你的 Cardputer **有** PSRAM(ADV 版),NOR(1MB)直接放 PSRAM,只 ROM 走 SD,简单很多。

> 参考实现:`tab5-nc2000` 的 NAND 分页(`lib/wqx/nand.cpp` 的 LRU 块缓存 + 写回)
> 思路可直接套用到 ROM/NOR;rainyx 的 `M5Cardputer-NC1020-Emu/src/nc1020.cpp`
> `readRomBank`/`nor_temp_file` 是 Cardputer 上的现成范例。

## 键盘

Cardputer 内置 QWERTY,`src/main.cpp` 的 `kb_map[4][14]`(沿用 rainyx 版的 NC1020
矩阵码)→ `SetKeyWayback(code%8, code/8, down)`(IO_V2 扫描读的 `keypadmatrix`)。
Fn 键(`0x00`)按住时第一排变应用键(英汉/计算/…)。电源键 `0x0f` 走独立引脚
`ram_io[0x0b]`。**无触摸,无软键盘**,全靠物理键。

## 显示

160×80 LCD → `M5Canvas`(RGB565)逐像素填(1bpp 黑白 / 2bpp 四级灰度 GREY4)→
`pushRotateZoom` 拉伸到 240×135 面板。无旋转无 PPA。

## 编译/烧录

```
# VSCode 装 PlatformIO 插件,打开 cardputer-nc1020/,Build/Upload
# 或 CLI:
pio run -t upload
pio device monitor
```
SD 卡根目录放 `nc1020.rom` + `nc1020.nor`(wqx 格式)。
