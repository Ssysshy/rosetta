# 项目长期记忆：rosetta

## 工程性质
ESP-IDF v5.5.4 工程，目标芯片 **esp32s3**（M5Stack StopWatch UserDemo）。
Shell 中 `project()` 名称为 `StopWatch-UserDemo`，主固件即 `build/StopWatch-UserDemo.bin`。

## 构建环境（本机）
- ESP-IDF 位于 `/Users/admin/Documents/work/own/esp-idf`（不是 `~/esp/esp-idf`），每个新终端要先：
  `. /Users/admin/Documents/work/own/esp-idf/export.sh`
- 工具链在 `~/.espressif`；`cmake` / `ninja` 装在 IDF 的 venv
  （`/Users/admin/.espressif/python_env/idf5.5_py3.13_env/bin`），已随 export.sh 生效。

## 构建流程
1. `python3 ./fetch_repos.py`  —— 从 GitHub 拉第三方组件到 `components/`（mooncake、M5GFX、lvgl、M5IOE1、M5PM1、ArduinoJson、BMI270 等），并对 M5IOE1/M5PM1 打 `patches/` 补丁。
2. `idf.py set-target esp32s3`
3. `idf.py build`
4. 整包：`esptool --chip esp32s3 merge_bin -o <out>.bin --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0xd000 ota_data_initial.bin 0x20000 StopWatch-UserDemo.bin`

## 分区/Flash（partitions.csv + sdkconfig.defaults）
16MB Flash、8 线 PSRAM 80MHz、CPU 240MHz；双 OTA（ota_0/ota_1 各 0x4f0000），storage 4M。

## 注意
- `fetch_repos.py` 报 patch "skipped" 是正常的（补丁已存在）。
- 组件代码在 `components/` 内有本地改动（M5IOE1/M5PM1 的 i2c 兼容补丁），不要 `git checkout` 掉。
