#!/bin/bash
# INNO-MAKER gs_usb -> can0
BITRATE=1000000
RESTART_MS=100      # BUS-OFF 后自动恢复间隔(ms)
CHECK_SEC=3         # candump 自检时长(秒)

if ! ip link show can0 &> /dev/null; then
    echo "[✗] can0 不存在,检查 INNO-MAKER 有没有插好 (lsusb 找 1d50:606f)"
    exit 1
fi

# 干净重置 + 配波特率 + 开自动恢复
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate $BITRATE restart-ms $RESTART_MS
sudo ip link set can0 up

STATE=$(ip -details link show can0 | grep -oP 'state \K\S+')
echo "[✓] can0 已启动 @ ${BITRATE}bps (restart-ms=${RESTART_MS}, state=${STATE})"

# candump 自检:抓 CHECK_SEC 秒,数收到多少帧
echo "[..] 抓 ${CHECK_SEC}s 检测有无数据 (确认传感器已上电)..."
FRAMES=$(timeout ${CHECK_SEC} candump -n 100 can0 2>/dev/null | wc -l)

if [ "$FRAMES" -gt 0 ]; then
    echo "[✓] 收到 ${FRAMES} 帧,链路正常"
else
    echo "[✗] ${CHECK_SEC}s 内 0 帧。排查:"
    echo "     - 传感器有没有上电"
    echo "     - 波特率对不对 (当前 ${BITRATE},uSkin 不是 1M 就改脚本 BITRATE)"
    echo "     - 接线/终端电阻 (断电量 CAN_H-CAN_L 应 ~60Ω)"
    echo "     错误计数:"
    ip -details -statistics link show can0 | grep -A1 'bus-errors'
fi
