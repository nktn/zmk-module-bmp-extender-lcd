# zmk-module-bmp-extender-lcd

**English summary**: A [west](https://docs.zephyrproject.org/latest/develop/west/index.html)
module for ZMK adding (a) `sharp,ls0xx-swvcom`, a Zephyr display driver for
Sharp LS0xx-family memory LCDs (Rohit Gujarathi's upstream `ls0xx.c`, plus
software VCOM toggling via the serial command V bit for wirings without an
`EXTCOMIN` line, 90/270 rotation implemented in the driver itself, row-level
dirty-diff SPI sending, a periodic self-healing full-panel resend, and an
optional black-fill init), and (b) two build snippets, `bmp-extender-lcd` and
`bmp-extender-lcd-vertical`, wiring a Sharp LS011B7DH03 (160x68) to the BMP
Boost Extender Mini's FFC port via an ffc2mip adapter and beekeeb breakout.
Apache-2.0, driver derived from Zephyr's `ls0xx` driver. See the tables below
for wiring, Kconfig, and DT binding details, and "使い方" for snippet order.

## これは何か

BMP Boost (nRF52840) 向け ZMK モジュール。以下の 2 つを提供する。

1. **`sharp,ls0xx-swvcom` ディスプレイドライバ** (`drivers/display/ls0xx_swvcom.c`)
   Zephyr 標準の `ls0xx.c` (Rohit Gujarathi 作、Apache-2.0) をベースに、
   `EXTCOMIN` 線が無い配線 (beekeeb ブレイクアウト 5P 等) 向けにシリアル
   コマンドの V ビットでソフトウェア VCOM 反転を行う。加えて:
   - **回転 (90°/270°) をドライバ内で実装** — Zephyr の LVGL グルーは
     `lv_display_set_rotation` を呼ばないため、`write()`/`get_capabilities()`
     で独自に実装している。160x68 パネルに対し論理解像度 72x160 を
     advertise し (68 は 8 の倍数でないための切り上げ、68〜71 列は
     デッドカラムとして読み捨て)、論理座標のバッファをパネル座標の
     シャドウフレームバッファへ転置してから送信する。180° は未実装
     (enum に無い)。
   - **行単位の dirty-diff 送信** — シャドウ FB と比較し、実際に内容が
     変化したパネル行の連続範囲だけを SPI 送信する。全行 clean なら
     送信自体をスキップする。
   - **自己修復用の周期フル送信** (`CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS`、
     既定 2000ms、0 で無効化) — パネル自体のビストーブルメモリが
     電圧サグ/EMI 等で自然に化けることがあるため、shadow FB の内容を
     周期的に全行 blind resend して補修する。
   - **黒フィル初期化** (`CONFIG_LS0XX_SWVCOM_INIT_FILL_BLACK`) — 起動直後の
     最初のフレームが黒の場合、白 CLEAR を経由せず直接黒を送ることで
     白フラッシュを避ける。

2. **スニペット `bmp-extender-lcd` / `bmp-extender-lcd-vertical`**
   BMP Boost Extender Mini の FFC ポートから Sharp LS011B7DH03 (160x68) を
   ffc2mip アダプタ + beekeeb ブレイクアウト経由で駆動するための配線・
   SPI/LVGL 既定値。前者が横表示 (無回転)、後者が縦表示 (回転90°) を追加する
   差分。

## 配線

overlay ヘッダコメント (`snippets/bmp-extender-lcd/bmp-extender-lcd.overlay`)
より、Extender Mini FFC ↔ BMP Boost 追加 IO の対応:

| FFC ピン | 用途 | BMP Boost ピン | 備考 |
|---|---|---|---|
| 2 | SCLK | P0.17 | SPIM1 |
| 3 | SDIO | P0.21 | 液晶へは書き込みのみ (MISO 未割当) |
| 4 | MOTION | P0.29 | 未使用 |
| 5 | CS | P0.31 | アクティブ High |
| 6 | VCC | P0.24 | GPIO 給電。`zmk,ext-power-generic` (`EXT_POWER` ノード)、`init-delay-ms = <5>` でパネル VDD 安定を待ってから初期化書き込み |

- SPIM1 を表示専用に使用 (トラックボールの spi0 とは独立)。
- SCLK/MOSI/CS のドライブ強度は高駆動 (`NRF_DRIVE_H0H1` / `NRF_GPIO_DRIVE_H0H1`)
  — ジャンパ配線への EMI 耐性を上げるため。
- SPI クロックは 1 MHz (`spi-max-frequency = <1000000>`)。
- `vcom-period-ms = <34>` (半周期 17ms、LS011B7DH03 の COM 周波数 約30Hz に対応)。

## 使い方

`west.yml` にモジュールとスニペットを追加する:

```yaml
manifest:
  remotes:
    - name: nktn
      url-base: https://github.com/nktn
  projects:
    - name: zmk-module-bmp-extender-lcd
      remote: nktn
      revision: <固定 SHA を推奨>
```

固定 SHA へのピン留めを推奨 (再現性・改ざん検知のため)。ブランチ名指定は
避けること。

`build.yaml` の `snippet:` リストへ、**この順序を守って**追加する:

```yaml
snippet:
  - bmp-extender-lcd
  - bmp-extender-lcd-vertical   # 縦表示のみ。横表示なら省略
  - <アプリケーション自身のステータス画面スニペット>  # 例: tecla-cero の display-mip
```

### 順序制約 (重要)

`bmp-extender-lcd-vertical` は `bmp-extender-lcd.conf` の
`CONFIG_LV_Z_VDB_SIZE=5` を `100` に上書きし、`CONFIG_LV_Z_FULL_REFRESH=y`
を設定する。ZMK のビルドワークフロー (`west build -S`) は snippet を
`build.yaml` の記載順に連結し、`EXTRA_CONF_FILE` は**後に書いた方が勝つ**。
したがって:

- 正しい順序 (`bmp-extender-lcd` → `bmp-extender-lcd-vertical`):
  `VDB_SIZE=100` + `FULL_REFRESH=y` (整合、コントローラ側に常に完全な
  フレームバッファがある前提を満たす)
- 逆順にすると `VDB_SIZE=5` が後勝ちし、`FULL_REFRESH=y` なのに
  全画面未満のバッファしか渡らない危険な構成が黙って成立してしまう。
  これはドライバの `BUILD_ASSERT` (`drivers/display/ls0xx_swvcom.c`) が
  コンパイル時に検出して拒否する。

アプリケーション自身のステータス画面 conf/overlay (ステータス表示の内容、
レイアウトなど製品層の設定) は、この 2 つのスニペットの**後ろ**に置くこと。

横表示 (無回転) のみで使う場合は `bmp-extender-lcd` だけを追加すればよい。

Zephyr のモジュール snippet 解決は `zephyr/module.yml` の `snippet_root`
で行われる (Zephyr 3.4 以降で対応。ZMK の main ブランチは Zephyr 4.1 を
使用)。本モジュールの `zephyr/module.yml` は `snippet_root: .` を指定して
いるため、`snippets/` 配下の各スニペットが自動的に解決される。

## Kconfig

| シンボル | 既定値 | 説明 |
|---|---|---|
| `CONFIG_LS0XX_SWVCOM` | y (`DT_HAS_SHARP_LS0XX_SWVCOM_ENABLED` かつ `DISPLAY` 依存) | ドライバ本体の有効化 |
| `CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS` | 2000 | 自己修復用の周期フル送信間隔 (ms)。0 で無効化。回転有効時のみ効果あり |
| `CONFIG_LS0XX_SWVCOM_INIT_FILL_BLACK` | n | 初期化時に CLEAR (白) の代わりに全黒フィルを送る。回転有効時のみ効果あり |

`bmp-extender-lcd.conf` が設定する CONFIG (抜粋、コメントより理由付き):

| CONFIG | 値 | 理由 |
|---|---|---|
| `CONFIG_ZMK_DISPLAY` | y | 表示機能を有効化 |
| `CONFIG_ZMK_EXT_POWER` | y | `EXT_POWER` (`zmk,ext-power-generic`, P0.24) でパネル電源をディープスリープ時に安全に落とすため必須 (フォークによっては Kconfig 既定値が無く既定 n) |
| `CONFIG_ZMK_DISPLAY_INVERT` | y | ZMK の意味論上は白文字/黒背景化だが、本構成では表示パイプラインの極性が逆で実機は黒文字/白(銀)背景になる (反射型 MIP はこちらが視認性が良い) |
| `CONFIG_ZMK_DISPLAY_WORK_QUEUE_DEDICATED` | y | 表示描画専用のワークキューに分離し、描画が刺さってもキー入力/BLE/ログを延命させる (choice の子シンボル) |
| `CONFIG_ZMK_DISPLAY_DEDICATED_THREAD_STACK_SIZE` | 4096 | LVGL9 の描画余裕のため既定 3072 から増量 |
| `CONFIG_LV_Z_MEM_POOL_SIZE` | 16384 | LVGL9 のメモリ要求増 (`LV_DRAW_LAYER_SIMPLE_BUF_SIZE` 等) に対応、既定 4096 では `lv_malloc` 失敗の恐れ |
| `CONFIG_LV_DRAW_SW_I1_LUM_THRESHOLD` | 100 | LVGL9 の 1bpp(I1) 二値化しきい値。既定 127 は実質フル被覆要求でフォントが痩せるため引き下げ |
| `CONFIG_LV_Z_VDB_SIZE` | 5 (`bmp-extender-lcd-vertical` で 100 に上書き) | 描画バッファを画面の約 5% に縮小し SPI トランザクションを分割、将来のバス共有時の占有を抑制 |
| `CONFIG_LV_DPI_DEF` | 161 | パネルの実 DPI |
| `CONFIG_LV_Z_BITS_PER_PIXEL` / `CONFIG_LV_COLOR_DEPTH_1` | 1 / y | 1bpp モノクロ描画 |

`bmp-extender-lcd-vertical.conf` が追加設定する CONFIG:

| CONFIG | 値 | 理由 |
|---|---|---|
| `CONFIG_LV_Z_FULL_REFRESH` | y | 回転経路では論理座標の部分更新でも転置後はパネル側のほぼ全域に影響しうるため、LVGL 側の描画を毎フレーム全画面 flush に揃える (実際の SPI 送信はドライバの行単位 dirty-diff でさらに絞られる) |
| `CONFIG_LV_Z_VDB_SIZE` | 100 | `FULL_REFRESH=y` に対応する全フレーム分のバッファ (**`bmp-extender-lcd` の後に置くこと、順序制約を参照**) |

## DT バインディングプロパティ (`sharp,ls0xx-swvcom`)

| プロパティ | 型 | 既定 | 説明 |
|---|---|---|---|
| `vcom-period-ms` | int | 34 | VCOM 極性反転の全周期 [ms]。半周期ごとに V ビットをトグル |
| `disp-en-gpios` | phandle-array | (任意) | DISPLAY enable ピン (配線されている場合のみ) |
| `rotation` | int (enum: 0/90/270) | 0 | 論理表示の回転角度 (時計回り)。0 は無回転で完全にゼロオーバーヘッド。180 は未実装 (enum に無い) |

**単一インスタンスのみ**: ドライバは `DT_INST_COMPAT_GET_ANY_NUM` ではなく
固定で `DT_INST 0` を前提にしている (シャドウ FB・dirty フラグ等がすべて
静的単一インスタンス変数)。2 台目のパネルを同時に駆動する用途では動かない。

## 制約・注意事項

- **単一インスタンス限定** (上記)。
- **実機検証は nRF52840 BMP Boost + LS011B7DH03、AAA 電池駆動 (VDD 約2.4V)
  のみ**。他の電圧・パネルでの動作は未検証。
- パネル自体のビストーブルメモリが電圧サグや EMI で自然に化ける事象が
  実機で確認されており (対策として `CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS`
  の周期自己修復を追加)、ソフトウェア側で発生源そのものを消すことは
  できない。
- tecla-cero での左側 (peripheral) 使用時は、キーボード側の overlay で
  競合する `spi3` を無効化する必要がある (アプリ側の関心事であり、本
  モジュール側の対応は不要)。

## ライセンス

Apache-2.0。ディスプレイドライバは Zephyr 標準の `ls0xx` ドライバ
(Copyright (c) 2020 Rohit Gujarathi) を元にした派生物。
