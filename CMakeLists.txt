# SPDX-License-Identifier: Apache-2.0
#
# zmk-module-bmp-extender-lcd: Sharp メモリ液晶 (LS0xx 系) をソフトウェア
# VCOM 反転で駆動するディスプレイドライバ。BMP Boost Extender Mini の FFC
# 経由の配線 (snippets/) と組で使う。

if(CONFIG_LS0XX_SWVCOM)
  zephyr_library()
  zephyr_library_sources(drivers/display/ls0xx_swvcom.c)
endif()
