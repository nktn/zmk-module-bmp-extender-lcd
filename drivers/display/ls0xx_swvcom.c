/*
 * Sharp LS0xx メモリ液晶ドライバ (ソフトウェア VCOM 反転付き)
 *
 * Zephyr 標準の drivers/display/ls0xx.c (Copyright (c) 2020 Rohit Gujarathi,
 * Apache-2.0) をベースに、EXTCOMIN 線が無い配線 (beekeeb ブレイクアウト 5P) 向けの
 * ソフトウェア VCOM 反転を追加したもの。
 *
 * Sharp MIP は液晶への DC バイアスを避けるため VCOM の周期反転が必須。
 * LS011B7DH03 の仕様は COM 周波数 約30Hz (極性切替 約17ms ごと) で、
 * 「1Hz で良い」系のパネルではない点に注意 (codex レビュー 2026-07-25)。
 * EXTCOMIN が配線されていないため、シリアルコマンドの V ビットを周期反転する。
 *
 * 排他制御: Zephyr の SPI_LOCK_ON は spi_config ポインタ単位の所有権のため、
 * 同じ spi_dt_spec を使う VCOM ワークと表示更新は互いに割り込めてしまう。
 * ドライバ専用の k_mutex で「V ビット状態の読み書き + SPI 転送シーケンス全体 +
 * SCS Low 期間の確保」を不可分に保護する。
 *
 * 既知の制約: ZMK のディープスリープ (system-off) 中はこのワークも止まり VCOM 反転は
 * 停止するが、System OFF 突入 (sys_poweroff()) 前に
 * snippets/bmp-extender-lcd/bmp-extender-lcd.overlay の zmk,ext-power-generic (P0.24) が
 * ZMK の zmk_pm_suspend_devices() 経由 PM_DEVICE_ACTION_SUSPEND でパネル電源自体を
 * 落とすため、VCOM 停止中に液晶へ通電されたままになる DC バイアス問題は生じない
 * (2026-08-02 codex P1 指摘対応。旧実装は P0.24 を gpio-hog で常時 ON 固定していた。
 * 詳細は tecla-cero キーボードリポジトリ (zmk-keyboard-tecla-cero-adv-private) の
 * snippets/display-mip/DESIGN.md 参照。同ファイルはこのモジュールには同梱していない)。
 * 起床はフルリセットなので本ドライバも ext-power も自然に再初期化される。
 *
 * 2026-08-02: Zephyr v4.1.0+zmk-fixes ベースへ移植。upstream の新ベースは
 * `serial-vcom-inversion` DT プロパティによるネイティブ SW VCOM 反転 (k_sem 制の
 * バス排他 + ls0xx_cmd() 内での暗黙トグル、既定周期 1000ms) を追加したが、本ドライバは
 * 採用しない: LS011B7DH03 の実仕様 (半周期 17ms) に対して精度・周期制御が粗く、
 * 書き込みコマンド自体も呼ぶたびに V ビットをトグルする設計のため意図した波形を
 * 保証できない。上記の専用 k_mutex による排他 (このドライバ独自の設計、
 * codex レビュー 2026-07-25 で堅牢性確認済み) を維持し、DEVICE_API マクロ化・
 * display_driver_api の未使用フィールド省略 (read/get_framebuffer/set_brightness/
 * set_contrast/set_orientation は新ベースでも未実装、struct 自体は変更無しで
 * NULL 許容) のみ新ベースに追従する。
 *
 * 2026-08-03: 縦表示 (portrait) 対応で rotation プロパティを追加。
 * Zephyr v4.1 の LVGL グルー (modules/lvgl/lvgl.c, lvgl_display_mono.c) には
 * 回転サポートが無い (lv_display_set_rotation を呼ばない。flush は
 * display_write へ座標を素通しするだけ) ため、回転は本ドライバの
 * write()/get_capabilities() で独自に実装する。詳細な設計判断・実測値は
 * tecla-cero キーボードリポジトリ (zmk-keyboard-tecla-cero-adv-private) の
 * snippets/display-mip/DESIGN.md「縦表示」節 (このモジュールには同梱していない)
 * を参照。ここでは要点のみ:
 *   - rotation=0 (既定) の経路は完全に無変更 (下記 #if 群はすべて
 *     LS0XX_ROTATION == 0 のとき事実上消える)
 *   - rotation=90/270 のとき get_capabilities() は論理解像度 72x160 を
 *     advertise する。パネル高さ 68 は 8 の倍数でないため、
 *     modules/lvgl/lvgl.c の set_px_at_pos() (buf + x/8 + y*width/8 という
 *     整数演算で 1bpp パックする) にそのまま width=68 を渡すと
 *     68/8=8.5 が 8 に切り捨てられ行毎に半バイトずれて破綻する
 *     (ソース読解で確認、実機検証ではない)。よって 8 の倍数へ切り上げた
 *     72 を advertise し、68〜71 列 (デッドカラム) は write() 側で読み捨てる
 *   - write() は論理座標の MONO01 バッファを転置し、パネル空間 (160x68) の
 *     シャドウバッファへ書く。送信するパネル行の範囲は dirty 判定に従う
 *     (2026-08-04 追記、下記参照)
 *   - VCOM 反転 (k_mutex + k_work) や ls0xx_clear()/ls0xx_init() は無変更。
 *     回転は write()/get_capabilities() だけに閉じている
 *
 * 2026-08-04: 実機フィードバックで「ホスト (iPhone) 不在中に画面が
 * チラつき続ける」不具合が判明。原因と推定される連鎖 (詳細はアプリ側の
 * 出力先ウィジェット実装 (例: tecla-cero の src/display/endpoint_refresh.c
 * 冒頭コメント) と、tecla-cero キーボードリポジトリの DESIGN.md 参照):
 * 出力先ウィジェット等の再描画イベントが内容不変でも発生し得る →
 * lv_label_set_text() は新旧テキストを比較せず常に invalidate する →
 * FULL_REFRESH モードでは任意の invalidation が全画面再描画になる →
 * (旧実装) 縦表示ドライバが内容不変でも毎回パネル全68行を re-send して
 * いたため、SPI 送信と VCOM 反転 (半周期17ms、mutex排他) のタイミングが
 * ぶつかりチラつきとして見えていた。対策として
 * ls0xx_transpose_into_shadow() で書き込み前後のビットを比較し、実際に
 * 変化したパネル行だけ ls0xx_row_dirty[] に記録するようにした。
 * ls0xx_write() はこれを見て dirty な行の連続範囲だけを
 * ls0xx_update_display() で送信し、**全行 clean なら SPI 送信を完全に
 * スキップする**。旧来の「縦論理の部分更新は転置後ほぼ全域に影響するため
 * 部分送信の最適化はしない」という判断は撤回し、行単位の実差分送信に
 * 変更した (実装が複雑になる代わりに、内容不変の再描画イベントで
 * パネルに一切触れなくなる。副次効果として将来トラックボールと SPI
 * バスを共有する改版基板でのバス占有も、変化があった行数に比例する形に
 * 縮小される)
 *
 * 2026-08-04 (codex レビュー 5巡目 P2): 上記の差分送信中に SPI 送信が
 * 失敗すると、既に転置済みの shadow がパネルの実際の内容より先行した
 * まま固定され、次回以降同じフレームが来ても「差分なし」と誤判定されて
 * 二度と送信されない問題が指摘された。ls0xx_shadow_valid フラグを追加し、
 * 送信失敗時は false にして次回 write() でパネル全行を強制送信 (成功で
 * true に復帰) するようにした。詳細は ls0xx_shadow_valid の定義コメント
 * 参照。あわせて診断カウンタ (dirty_rows_sent) を送信成功が確定した分
 * だけ加算するよう修正した (以前は ls0xx_update_display() の戻り値を
 * 見る前に加算していた)
 *
 * 2026-08-05: 実機写真で「ホスト不在中に画面全体へランダムな斑点・横筋
 * ノイズが蓄積する」不具合が判明 (差分送信導入前は頻繁な全画面再描画が
 * 化けを毎回修復していたため「チラつき」に見え、差分送信導入後は内容
 * 不変なら送信しないため化けが蓄積して残るようになった、という経緯。
 * 詳細は tecla-cero キーボードリポジトリの DESIGN.md「パネルメモリ化け」節
 * 参照)。根本原因はソフトウェア
 * の外側 (電圧サグ/EMI 疑い) と推定されファームでは消せないため、
 * shadow FB を周期的にパネルへ blind resend して自己修復する仕組み
 * (ls0xx_heal_work_handler()、CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS) を
 * 追加した
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sharp_ls0xx_swvcom

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ls0xx_swvcom, CONFIG_DISPLAY_LOG_LEVEL);

#include <stdbool.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LS0XX_PANEL_WIDTH   DT_INST_PROP(0, width)
#define LS0XX_PANEL_HEIGHT  DT_INST_PROP(0, height)

#define LS0XX_PIXELS_PER_BYTE  8U
#define LS0XX_BYTES_PER_LINE  ((LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE) + 2)

#define LS0XX_BIT_WRITECMD    0x01
#define LS0XX_BIT_VCOM        0x02
#define LS0XX_BIT_CLEAR       0x04

/* VCOM 極性反転の全周期 [ms]。半周期ごとに V ビットをトグルする */
#define LS0XX_VCOM_PERIOD_MS  DT_INST_PROP(0, vcom_period_ms)

/* SCS setup 6µs / hold 2µs (LS011B7DH03)。CS 制御の delay で setup/hold を確保 */
#define LS0XX_CS_DELAY_US 6
/* トランザクション間の SCS Low 最小期間 */
#define LS0XX_SCS_LOW_US 6

BUILD_ASSERT((LS0XX_PANEL_WIDTH % LS0XX_PIXELS_PER_BYTE) == 0,
	     "panel width must be a multiple of 8");
BUILD_ASSERT(LS0XX_VCOM_PERIOD_MS >= 2, "vcom-period-ms must be >= 2");

/* 回転角度 (時計回り, 度)。0 (既定) は無回転で下記の rotation 用コードは
 * すべてコンパイルアウトされる。180 は enum 自体に無い (dts binding 参照)
 */
#define LS0XX_ROTATION DT_INST_PROP(0, rotation)

BUILD_ASSERT(LS0XX_ROTATION == 0 || LS0XX_ROTATION == 90 || LS0XX_ROTATION == 270,
	     "rotation must be 0, 90, or 270 (180 not implemented, see README.md)");

#if LS0XX_ROTATION != 0
/* 論理 (LVGL 側) の横幅は 8bit 境界に切り上げた値を advertise する。
 * パネル高さ 68 は 8 の倍数でないため、68 をそのまま渡すと
 * modules/lvgl/lvgl.c の 1bpp パック演算 (x/8 + y*width/8) が整数除算で
 * 破綻する (68/8=8.5 が 8 に切り捨てられ行毎に半バイトずれる)。
 * 72 (=68 を 8 の倍数へ切り上げ) を advertise し、68〜71 列は
 * デッドカラムとして転置時に読み捨てる
 */
#define LS0XX_LOGICAL_WIDTH  (((LS0XX_PANEL_HEIGHT + LS0XX_PIXELS_PER_BYTE - 1) / \
			       LS0XX_PIXELS_PER_BYTE) * LS0XX_PIXELS_PER_BYTE)
#define LS0XX_LOGICAL_HEIGHT LS0XX_PANEL_WIDTH

#define LS0XX_SHADOW_BYTES_PER_LINE (LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE)

/* パネル空間 (160x68) のシャドウフレームバッファ。論理座標からの転置結果を
 * 保持し、ls0xx_update_display() へそのまま渡す (行レイアウトは
 * ls0xx_update_display() が読む data_buf と同じ: 1 行 = PANEL_WIDTH/8 バイト)
 */
static uint8_t ls0xx_shadow_fb[LS0XX_PANEL_HEIGHT * LS0XX_SHADOW_BYTES_PER_LINE];

/* 2026-08-04 実機フィードバック対応: パネル行 (yp, 0..67) 単位の dirty
 * フラグ。ls0xx_transpose_into_shadow() が実際に shadow のビットを変化
 * させた行だけ true にする (呼び出しごとにリセットされる、ある write()
 * 呼び出し1回分の dirty 状態)。ls0xx_write() はこれを見て変化した行の
 * 連続範囲だけを ls0xx_update_display() で送信し、全行 clean なら SPI
 * 送信を完全にスキップする。
 *
 * 背景: ホスト (iPhone) 不在中に出力先ウィジェット等の再描画が (内容不変
 * でも) 繰り返し発生すると、FULL_REFRESH モードでは毎回全画面が
 * invalidate → flush される。以前の実装は内容が変わっていなくても
 * 毎回パネル全68行を re-send しており、この SPI 送信と VCOM 反転
 * (半周期17ms、mutex排他) のタイミングがぶつかって画面のチラつきとして
 * 見えていた (実機フィードバック、詳細はアプリ側の出力先ウィジェット実装
 * (例: tecla-cero の src/display/endpoint_refresh.c 冒頭コメント) と、
 * tecla-cero キーボードリポジトリの DESIGN.md 参照)。
 * dirty 行判定により「実際に絵が変わった行だけ」送信するため、内容不変の
 * 再描画イベントは (行数分の transpose 計算はするが) SPI を一切
 * 占有しなくなり、VCOM 反転とのジッタが起きなくなる */
static bool ls0xx_row_dirty[LS0XX_PANEL_HEIGHT];

/* codex レビュー 2026-08-04 (5巡目 P2): SPI 送信失敗時、既に転置済みの
 * shadow がパネルの実際の内容より "先行" したまま固定される問題への対策。
 * このドライバはプロトコル上パネルから内容を読み戻す手段が無く、失敗した
 * 送信のロールバックもしない。よって送信が一部でも失敗すると、以後
 * ls0xx_transpose_into_shadow() の新旧比較は「shadow 同士の比較」でしか
 * なくなり、次回以降同じフレームが来ると (shadow は既に新しい内容なので)
 * 差分なしと判定されて二度と送信されず、パネルは古い内容のまま固定されて
 * しまう (複数 dirty range の途中で失敗した場合、未送信の後続 range も
 * 同様に取りこぼす)。
 *
 * 対策: この flag が false の間は ls0xx_write() が dirty 判定を無視して
 * パネル全 LS0XX_PANEL_HEIGHT 行を強制送信し、成功したら true に戻す
 * (次回のいずれかの送信が失敗したら再び false になる)。「送信に成功した
 * 範囲だけ shadow が正 (valid)」という committed/working の2面 shadow
 * 方式より単純だが、無効化中は差分最適化が一時的に効かなくなるだけで
 * 常に安全側 (全行再送) に倒れるため、この用途には十分。
 * ls0xx_row_dirty[] と同じく data->fb_lock 保護下でのみ読み書きする */
static bool ls0xx_shadow_valid = true;

/* codex レビュー P2-2 (2026-08-03): snippet の記載順を build.yaml で
 * bmp-extender-lcd の後ろに bmp-extender-lcd-vertical を置くこと、という
 * 運用ルールに依存して CONFIG_LV_Z_VDB_SIZE=100 を確保している
 * (bmp-extender-lcd-vertical.conf 参照)。記載順を誤ると bmp-extender-lcd.conf
 * の VDB_SIZE=5 が後勝ちし、FULL_REFRESH=y なのに全画面未満のバッファしか
 * 渡らない危険な構成が黙って成立してしまう。ビルド時に検出できるよう固定する
 * (2026-09-03: display-mip* からハード層 bmp-extender-lcd* への切り出しに
 * 伴い運用ルールの対象スニペット名を更新) */
BUILD_ASSERT(!IS_ENABLED(CONFIG_LV_Z_FULL_REFRESH) || CONFIG_LV_Z_VDB_SIZE >= 100,
	     "CONFIG_LV_Z_FULL_REFRESH=y requires CONFIG_LV_Z_VDB_SIZE >= 100 when "
	     "rotation is enabled (check snippet order: bmp-extender-lcd-vertical "
	     "must come after bmp-extender-lcd in build.yaml, see README.md)");
#endif /* LS0XX_ROTATION != 0 */

struct ls0xx_config {
	struct spi_dt_spec bus;
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	struct gpio_dt_spec disp_en_gpio;
#endif
};

struct ls0xx_data {
	const struct device *dev;
	struct k_work_delayable vcom_work;
	/* lock が保護する対象: vcom の読み書きと SPI 転送シーケンス全体。
	 * SPI_LOCK_ON は spi_config ポインタ単位の所有権のため、同一 spec を
	 * 使うコンテキスト同士の排他はこの mutex が担う
	 */
	struct k_mutex lock;
	/* 現在の VCOM 状態 (0 または LS0XX_BIT_VCOM) */
	uint8_t vcom;
#if LS0XX_ROTATION != 0
	/* codex レビュー P2-1 (2026-08-03): ls0xx_shadow_fb への転置開始から
	 * ls0xx_update_display() でのパネル全68行送信完了までを不可分に保護する。
	 * 現状の呼び出し元 (専用 display workqueue で display_write() を直列化)
	 * では並行呼び出しは起きないが、ドライバ単体としての堅牢性のために追加する。
	 *
	 * ロック順序: fb_lock (外側) → lock (内側、ls0xx_update_display() 内で
	 * 取得)。fb_lock を保持したまま lock を取る一方向のみで、逆方向 (lock を
	 * 保持したまま fb_lock を取る) は存在しないためデッドロックしない
	 */
	struct k_mutex fb_lock;
#if CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS > 0
	/* 自己修復リフレッシュ用の周期ワーク (2026-08-05 実機フィードバック
	 * 対応、ls0xx_heal_work_handler() 冒頭コメント参照) */
	struct k_work_delayable heal_work;
#endif
#endif
};

static int ls0xx_blanking_off(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	const struct ls0xx_config *config = dev->config;

	return gpio_pin_set_dt(&config->disp_en_gpio, 1);
#else
	LOG_WRN("Unsupported");
	return -ENOTSUP;
#endif
}

static int ls0xx_blanking_on(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	const struct ls0xx_config *config = dev->config;

	return gpio_pin_set_dt(&config->disp_en_gpio, 0);
#else
	LOG_WRN("Unsupported");
	return -ENOTSUP;
#endif
}

static int ls0xx_cmd(const struct device *dev, uint8_t *buf, uint8_t len)
{
	const struct ls0xx_config *config = dev->config;
	struct spi_buf cmd_buf = { .buf = buf, .len = len };
	struct spi_buf_set buf_set = { .buffers = &cmd_buf, .count = 1 };

	return spi_write_dt(&config->bus, &buf_set);
}

/* SPI 転送シーケンスの終端処理: CS 解放と SCS Low 期間の確保。
 * 呼び出し元は data->lock を保持していること
 */
static void ls0xx_end_transfer(const struct device *dev)
{
	const struct ls0xx_config *config = dev->config;

	spi_release_dt(&config->bus);
	k_busy_wait(LS0XX_SCS_LOW_US);
}

static void ls0xx_vcom_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ls0xx_data *data = CONTAINER_OF(dwork, struct ls0xx_data, vcom_work);
	const struct device *dev = data->dev;
	uint8_t cmd[2];
	uint8_t next;
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);
	next = data->vcom ^ LS0XX_BIT_VCOM;
	/* V ビットのみの維持コマンド (M0=0, M2=0) + トレーラ */
	cmd[0] = next;
	cmd[1] = 0;
	err = ls0xx_cmd(dev, cmd, sizeof(cmd));
	if (err == 0) {
		data->vcom = next;
	}
	ls0xx_end_transfer(dev);
	k_mutex_unlock(&data->lock);

	if (err) {
		LOG_WRN("VCOM toggle failed: %d", err);
	}

	k_work_reschedule(dwork, K_MSEC(LS0XX_VCOM_PERIOD_MS / 2));
}

/* LS0XX_SWVCOM_INIT_FILL_BLACK 有効ビルドでは init が黒転ブリッジ (全行黒送信)
 * に置き換わり未使用になるため __unused (ls0xx_init() の分岐コメント参照) */
static __unused int ls0xx_clear(const struct device *dev)
{
	struct ls0xx_data *data = dev->data;
	uint8_t clear_cmd[2];
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);
	clear_cmd[0] = LS0XX_BIT_CLEAR | data->vcom;
	clear_cmd[1] = 0;
	err = ls0xx_cmd(dev, clear_cmd, sizeof(clear_cmd));
	ls0xx_end_transfer(dev);
	k_mutex_unlock(&data->lock);

	return err;
}

#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
/* 2026-08-04 codex レビュー (4巡目、P2-2): usblog 縦ビルドで画面が
 * 完全に無表示になる不具合の実機切り分け用診断ログ。
 * 「flush がそもそも呼ばれているか / SPI 送信が成功しているか」を
 * 実機ログで判定できるようにする (毎フレームのスパムは避け、初回 +
 * 32回毎 + エラー時は毎回に絞る)。CONFIG_ZMK_USB_LOGGING が無効な
 * 通常ビルドではこのブロックごとコンパイルアウトされ、影響は無い。
 * 詳細は tecla-cero キーボードリポジトリ (zmk-keyboard-tecla-cero-adv-private) の
 * snippets/display-mip/DESIGN.md「usblog ビルドの画面無表示」節参照
 * (このモジュールには同梱していない) */
#define LS0XX_DIAG_LOG_INTERVAL 32
static uint32_t ls0xx_diag_update_calls;
#endif

static int ls0xx_update_display(const struct device *dev,
				uint16_t start_line,
				uint16_t num_lines,
				const uint8_t *data_buf)
{
	const struct ls0xx_config *config = dev->config;
	struct ls0xx_data *data = dev->data;
	uint8_t write_cmd[1];
	uint8_t ln = start_line;
	uint8_t dummy = 27;
	struct spi_buf line_buf[3] = {
		{
			.len = sizeof(ln),
			.buf = &ln,
		},
		{
			.len = LS0XX_BYTES_PER_LINE - 2,
		},
		{
			.len = sizeof(dummy),
			.buf = &dummy,
		},
	};
	struct spi_buf_set line_set = {
		.buffers = line_buf,
		.count = ARRAY_SIZE(line_buf),
	};
	int err;

	LOG_DBG("Lines %d to %d", start_line, start_line + num_lines - 1);

	k_mutex_lock(&data->lock, K_FOREVER);
	write_cmd[0] = LS0XX_BIT_WRITECMD | data->vcom;
	err = ls0xx_cmd(dev, write_cmd, sizeof(write_cmd));

	for (; err == 0 && ln <= start_line + num_lines - 1; ln++) {
		line_buf[1].buf = (uint8_t *)data_buf;
		err = spi_write_dt(&config->bus, &line_set);
		data_buf += LS0XX_PANEL_WIDTH / LS0XX_PIXELS_PER_BYTE;
	}

	if (err == 0) {
		/* 最終ラインの後のトレーラ 8bit (内容は不問) */
		err = ls0xx_cmd(dev, write_cmd, sizeof(write_cmd));
	}

	ls0xx_end_transfer(dev);
	k_mutex_unlock(&data->lock);

#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
	ls0xx_diag_update_calls++;
	if (err != 0 || ls0xx_diag_update_calls == 1 ||
	    (ls0xx_diag_update_calls % LS0XX_DIAG_LOG_INTERVAL) == 0) {
		LOG_INF("ls0xx_update_display: call#%u lines=%d..%d err=%d",
			ls0xx_diag_update_calls, start_line,
			start_line + num_lines - 1, err);
	}
#endif

	return err;
}

#if LS0XX_ROTATION != 0
/* 論理座標 (y0 起点、height 行、幅 LS0XX_LOGICAL_WIDTH の MONO01 バッファ) を
 * パネル空間 (160x68) のシャドウバッファへビット転置する。
 *
 * ビット値の意味はここでは問わない (0/1 をそのまま位置だけ変えてコピーする)。
 * lvgl.c の lvgl_transform_buffer() が MONO01 の極性を確定させた後の buf を
 * そのまま受け取るため、rotation=0 の経路 (バイト列を無変換で転送) と同様
 * 値の解釈はしなくてよい。
 *
 * 回転方向の定義 (時計回りにコンテンツを回して表示):
 *   90:  xp = PANEL_WIDTH-1-yl, yp = xl
 *   270: xp = yl,               yp = PANEL_HEIGHT - 1 - xl
 * 実機で天地/左右が逆であれば dts の rotation を 90⇔270 で入れ替えること。
 *
 * (2026-08-03 codex レビュー P1: 初版はこの 90/270 の式が入れ替わっていた。
 * 左上原点で論理→物理を時計回りに90°回す変換は xp=PANEL_WIDTH-1-yl, yp=xl
 * であることを標準的な画像回転式から導出して修正。binding
 * (dts/bindings/display/sharp,ls0xx-swvcom.yaml) と、tecla-cero キーボード
 * リポジトリの DESIGN.md の記述もこの式に合わせて修正済み)
 *
 * xl は 0..PANEL_HEIGHT-1 (=0..67) のみ処理し、デッドカラム
 * (68..LS0XX_LOGICAL_WIDTH-1) は読み捨てる
 *
 * 2026-08-04: 書き込み前の旧ビットと新ビットを比較し、実際に変化した
 * パネル行 (yp) だけを ls0xx_row_dirty[] に記録するようにした
 * (呼び出し元 ls0xx_write() がこの配列を見て送信範囲を決める。詳細は
 * ls0xx_row_dirty の定義コメント参照)。ls0xx_row_dirty[] のリセットは
 * 呼び出し元の責務 (この関数は立てるだけ、消さない) */
static void ls0xx_transpose_into_shadow(uint16_t y0, uint16_t height, const uint8_t *buf)
{
	const size_t src_bytes_per_row = LS0XX_LOGICAL_WIDTH / LS0XX_PIXELS_PER_BYTE;

	for (uint16_t r = 0; r < height; r++) {
		uint16_t yl = y0 + r;
		const uint8_t *src_row = buf + (size_t)r * src_bytes_per_row;

		for (uint16_t xl = 0; xl < LS0XX_PANEL_HEIGHT; xl++) {
			uint8_t src_bit = (src_row[xl / LS0XX_PIXELS_PER_BYTE] >>
					   (xl % LS0XX_PIXELS_PER_BYTE)) & 0x01U;
			uint16_t xp;
			uint16_t yp;

#if LS0XX_ROTATION == 90
			xp = LS0XX_PANEL_WIDTH - 1 - yl;
			yp = xl;
#else /* LS0XX_ROTATION == 270 */
			xp = yl;
			yp = LS0XX_PANEL_HEIGHT - 1 - xl;
#endif

			uint8_t *dst_byte = &ls0xx_shadow_fb[(size_t)yp * LS0XX_SHADOW_BYTES_PER_LINE +
							      xp / LS0XX_PIXELS_PER_BYTE];
			uint8_t mask = (uint8_t)(1U << (xp % LS0XX_PIXELS_PER_BYTE));
			bool old_bit = (*dst_byte & mask) != 0;

			if ((src_bit != 0) != old_bit) {
				ls0xx_row_dirty[yp] = true;
			}

			if (src_bit) {
				*dst_byte |= mask;
			} else {
				*dst_byte &= (uint8_t)~mask;
			}
		}
	}
}
#endif /* LS0XX_ROTATION != 0 */

static int ls0xx_write(const struct device *dev, const uint16_t x,
		       const uint16_t y,
		       const struct display_buffer_descriptor *desc,
		       const void *buf)
{
	LOG_DBG("X: %d, Y: %d, W: %d, H: %d", x, y, desc->width, desc->height);

#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
	/* 2026-08-04 codex レビュー (4巡目、P2-2) 診断ログ:
	 * ls0xx_update_display() 側のログと合わせて「flush (display_write) が
	 * そもそも呼ばれているか」を実機で確認できるようにする。詳細は
	 * ls0xx_diag_update_calls のコメント参照 */
	{
		static uint32_t diag_write_calls;

		diag_write_calls++;
		if (diag_write_calls == 1 ||
		    (diag_write_calls % LS0XX_DIAG_LOG_INTERVAL) == 0) {
			LOG_INF("ls0xx_write: call#%u x=%d y=%d w=%d h=%d", diag_write_calls, x,
				y, desc->width, desc->height);
		}
	}
#endif

	if (buf == NULL) {
		LOG_WRN("Display buffer is not available");
		return -EINVAL;
	}

#if LS0XX_ROTATION != 0
	if (desc->width != LS0XX_LOGICAL_WIDTH) {
		LOG_ERR("Width must be %d (rotated logical width)", LS0XX_LOGICAL_WIDTH);
		return -EINVAL;
	}

	if (desc->pitch != desc->width) {
		LOG_ERR("Unsupported mode");
		return -ENOTSUP;
	}

	if ((y + desc->height) > LS0XX_LOGICAL_HEIGHT) {
		LOG_ERR("Buffer out of bounds (height)");
		return -EINVAL;
	}

	if (x != 0) {
		LOG_ERR("X-coordinate has to be 0");
		return -EINVAL;
	}

	/* codex レビュー P3 (2026-08-03): ls0xx_transpose_into_shadow() は
	 * buf から DIV_ROUND_UP(pitch * height, 8) バイトを読む。desc->buf_size
	 * がそれ未満だと範囲外読み出しになるため事前検証する */
	{
		size_t required = DIV_ROUND_UP((size_t)desc->pitch * desc->height, 8);

		if (desc->buf_size < required) {
			LOG_ERR("Buffer too small: buf_size=%u required=%zu",
				desc->buf_size, required);
			return -EINVAL;
		}
	}

	{
		struct ls0xx_data *data = dev->data;
		int err = 0;
#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
		uint16_t dirty_rows_sent = 0;
#endif

		/* codex レビュー P2-1: 転置開始から全dirty行送信完了までを
		 * fb_lock で保護する (ロック順序は struct ls0xx_data の
		 * fb_lock フィールドコメント参照) */
		k_mutex_lock(&data->fb_lock, K_FOREVER);

		/* 2026-08-04 実機フィードバック対応: 今回の呼び出し分の dirty
		 * フラグをリセットしてから転置する (前回までの dirty 状態を
		 * 引きずらない。転置自体は変化した行だけ true を立てる。
		 * ls0xx_row_dirty の定義コメント参照) */
		memset(ls0xx_row_dirty, 0, sizeof(ls0xx_row_dirty));
		ls0xx_transpose_into_shadow(y, desc->height, buf);

		if (!ls0xx_shadow_valid) {
			/* 前回以前の送信失敗で shadow がパネルの実際の内容より
			 * 先行している可能性があるため、今回の dirty 判定は無視
			 * してパネル全行を強制送信する (ls0xx_shadow_valid
			 * 定義コメント参照)。成功して初めて valid に戻す */
			err = ls0xx_update_display(dev, 1, LS0XX_PANEL_HEIGHT, ls0xx_shadow_fb);
			if (err == 0) {
				ls0xx_shadow_valid = true;
#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
				dirty_rows_sent = LS0XX_PANEL_HEIGHT;
#endif
			}
		} else {
			/* dirty な行の連続範囲ごとに送信する。Sharp MIP プロトコルは
			 * 任意の開始行を指定できるため、ls0xx_update_display() をそのまま
			 * 複数回呼べる。**全行 clean なら一度も呼ばれず SPI 送信を完全に
			 * スキップする** (画面内容が変わっていない再描画イベントで
			 * パネルに触れない = チラつきの直接原因である SPI/VCOM 競合が
			 * 起きなくなる) */
			for (uint16_t row = 0; row < LS0XX_PANEL_HEIGHT;) {
				if (!ls0xx_row_dirty[row]) {
					row++;
					continue;
				}

				uint16_t range_start = row;

				while (row < LS0XX_PANEL_HEIGHT && ls0xx_row_dirty[row]) {
					row++;
				}

				uint16_t num_lines = row - range_start;

				err = ls0xx_update_display(
					dev, range_start + 1, num_lines,
					&ls0xx_shadow_fb[(size_t)range_start * LS0XX_SHADOW_BYTES_PER_LINE]);
				if (err != 0) {
					/* 未送信の range が残ったままパネルと shadow が
					 * 食い違う。次回 write() で全行強制送信させる */
					ls0xx_shadow_valid = false;
					break;
				}

#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
				dirty_rows_sent += num_lines;
#endif
			}
		}

		k_mutex_unlock(&data->fb_lock);

#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
		/* diag_write_calls と同じ間引きポリシー (初回+32回毎+エラー時毎回)。
		 * dirty_rows_sent=0 の行が並ぶことが「チラつき対策で SPI を
		 * スキップできている」ことの実機ログ上の証拠になる */
		{
			static uint32_t diag_dirty_calls;

			diag_dirty_calls++;
			if (err != 0 || diag_dirty_calls == 1 ||
			    (diag_dirty_calls % LS0XX_DIAG_LOG_INTERVAL) == 0) {
				LOG_INF("ls0xx_write: call#%u dirty_rows_sent=%u/%u err=%d",
					diag_dirty_calls, dirty_rows_sent, LS0XX_PANEL_HEIGHT, err);
			}
		}
#endif

		return err;
	}
#else
	if (desc->width != LS0XX_PANEL_WIDTH) {
		LOG_ERR("Width not a multiple of %d", LS0XX_PANEL_WIDTH);
		return -EINVAL;
	}

	if (desc->pitch != desc->width) {
		LOG_ERR("Unsupported mode");
		return -ENOTSUP;
	}

	if ((y + desc->height) > LS0XX_PANEL_HEIGHT) {
		LOG_ERR("Buffer out of bounds (height)");
		return -EINVAL;
	}

	if (x != 0) {
		LOG_ERR("X-coordinate has to be 0");
		return -EINVAL;
	}

	/* パネルのライン番号は 1 始まり */
	return ls0xx_update_display(dev, y + 1, desc->height, buf);
#endif /* LS0XX_ROTATION != 0 */
}

#if LS0XX_ROTATION != 0 && CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS > 0
/* 自己修復リフレッシュ (2026-08-05 実機フィードバック対応): ホスト
 * (iPhone) 不在中に画面全体へランダムな斑点・横筋ノイズが蓄積する不具合が
 * 実機写真で確認された。解析:
 *   - 行単位 dirty 差分送信 (2026-08-04) 導入前は、内容不変でも頻繁な
 *     全画面再描画イベントのたびにパネル全68行を re-send していたため、
 *     化けたピクセルもその都度正しい内容で上書きされ「気付かないうちに
 *     直っていた」。この頻繁な re-send 自体が SPI/VCOM 競合を起こし
 *     「チラつき」に見えていた (2026-08-04 の対策対象)
 *   - dirty 差分送信導入後は、shadow FB の内容 (LVGL が意図した内容) が
 *     変化しない限り一切送信しなくなったため、化けたピクセルはパネル側に
 *     居座ったままになる。「チラつき」は無くなったが、化けたゴミが
 *     蓄積して残るようになった
 *   - 根本原因はソフトウェアの外側 (最有力候補: ①2.4V 仕様外駆動 + BLE
 *     アドバタイジングの無線電流スパイクによる電圧サグ、②試作機の
 *     ジャンパ配線への 2.4GHz EMI 誘導。詳細は tecla-cero キーボード
 *     リポジトリの DESIGN.md「パネルメモリ化け」節参照) と推定され、
 *     ファームウェアだけでは
 *     発生源を消せない
 *
 * 対策: shadow FB (LVGL が意図した「正しい」内容を常に保持している) を
 * そのまま周期的にパネル全 LS0XX_PANEL_HEIGHT 行へ blind resend する。
 * dirty 判定や ls0xx_shadow_valid の状態に関係なく常に送るため、実際に
 * 化けたピクセルがあってもこのタイマーの周期以内に必ず正しい内容で
 * 上書きされる (自己修復)。同一内容の再送は MIP プロトコル上無害
 * (同じ値で上書きされるだけで見た目は変化しない) なので、常時動かして
 * 副作用は無い。周期は CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS (既定2000ms、
 * 0 でこのタイマー自体が丸ごとコンパイルアウトされる)。電力面: ディープ
 * スリープ (System OFF) 中は ext-power がパネル電源ごと落とすため
 * (snippets/bmp-extender-lcd/bmp-extender-lcd.overlay 参照) このタイマーを
 * 止める特別な配慮は不要 (System OFF 中は k_work のスケジューラ自体が
 * 動かない)。通常動作中は 2000ms に一度の ~12ms SPI 転送
 * (bmp-extender-lcd-vertical.conf のコスト概算コメント参照) で、消費電力への
 * 影響は誤差の範囲
 */
static void ls0xx_heal_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ls0xx_data *data = CONTAINER_OF(dwork, struct ls0xx_data, heal_work);
	int err;

	k_mutex_lock(&data->fb_lock, K_FOREVER);
	err = ls0xx_update_display(data->dev, 1, LS0XX_PANEL_HEIGHT, ls0xx_shadow_fb);
	if (err == 0) {
		ls0xx_shadow_valid = true;
	}
	k_mutex_unlock(&data->fb_lock);

	if (err) {
		LOG_WRN("Heal refresh failed: %d", err);
	}

	k_work_reschedule(dwork, K_MSEC(CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS));
}
#endif /* LS0XX_ROTATION != 0 && CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS > 0 */

static void ls0xx_get_capabilities(const struct device *dev,
				   struct display_capabilities *caps)
{
	memset(caps, 0, sizeof(struct display_capabilities));
#if LS0XX_ROTATION != 0
	/* 論理解像度 (72x160) を advertise。68x160 ではなく 72 な理由は
	 * LS0XX_LOGICAL_WIDTH の定義コメント参照 (8bit境界のデッドカラム対策) */
	caps->x_resolution = LS0XX_LOGICAL_WIDTH;
	caps->y_resolution = LS0XX_LOGICAL_HEIGHT;
#else
	caps->x_resolution = LS0XX_PANEL_WIDTH;
	caps->y_resolution = LS0XX_PANEL_HEIGHT;
#endif
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01;
	caps->current_pixel_format = PIXEL_FORMAT_MONO01;
	caps->screen_info = SCREEN_INFO_X_ALIGNMENT_WIDTH;
}

static int ls0xx_set_pixel_format(const struct device *dev,
				  const enum display_pixel_format pf)
{
	if (pf == PIXEL_FORMAT_MONO01) {
		return 0;
	}

	LOG_ERR("not supported");
	return -ENOTSUP;
}

static int ls0xx_init(const struct device *dev)
{
	const struct ls0xx_config *config = dev->config;
	struct ls0xx_data *data = dev->data;
	int err;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	if (!gpio_is_ready_dt(&config->disp_en_gpio)) {
		LOG_ERR("DISP port device not ready");
		return -ENODEV;
	}
	LOG_INF("Configuring DISP pin to OUTPUT_HIGH");
	err = gpio_pin_configure_dt(&config->disp_en_gpio, GPIO_OUTPUT_HIGH);
	if (err) {
		LOG_ERR("DISP pin configure failed: %d", err);
		return err;
	}
#endif

	data->dev = dev;
	data->vcom = 0;
	k_mutex_init(&data->lock);
#if LS0XX_ROTATION != 0
	k_mutex_init(&data->fb_lock);
#if CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS > 0
	k_work_init_delayable(&data->heal_work, ls0xx_heal_work_handler);
#endif
#endif
	k_work_init_delayable(&data->vcom_work, ls0xx_vcom_work_handler);

#if LS0XX_ROTATION != 0 && IS_ENABLED(CONFIG_LS0XX_SWVCOM_INIT_FILL_BLACK)
	/* 黒転ブリッジ (CONFIG_LS0XX_SWVCOM_INIT_FILL_BLACK、2026-08-14 実機
	 * フィードバック対応): MIP は電源断後も画素メモリの残留電荷で前回画面を
	 * 保持し、ON 直後の ~1 秒 (Adafruit ブートローダ実行中) はファーム側から
	 * 消す手段が無い。その後ここで白クリア → アプリの初期フレーム描画、と
	 * 遷移すると「前回画面 → 白 → (初期フレームが黒なら) 黒」と脈絡なく
	 * 見えるため、CLEAR コマンド (白) は送らず、初期化書き込みを全行黒
	 * (インク) の送信に置き換える (全 68 行の書き込みは CLEAR と同様に
	 * 全画素メモリを既知状態に確定させるので残像対策としても等価。白を
	 * 経由しないぶん一瞬の白点滅も無い)。アプリケーション側 (キーボード側)
	 * が先頭フレーム全黒の起動アニメーションを持つ場合、その Kconfig が
	 * 本シンボルを select することで「前回画面 → 黒転 → アニメ」の
	 * ハードカットにつなげられる (例: tecla-cero では
	 * TECLA_DISPLAY_BOOT_ANIM が select する)。
	 * バイト値の意味: shadow_fb の 0 = CLEAR 後の白 (銀) 面
	 * (ls0xx_heal_work_handler の初回スケジュールコメント参照) なので、
	 * 黒 = 全ビット 1 = 0xff。送信後は shadow (0xff) = パネル (黒) が
	 * 一致しているので shadow_valid は true のままでよい。
	 * ロック: この時点では表示スレッド・heal タイマーとも未開始で
	 * fb_lock の競合相手はいない (VCOM ワークも下でスケジュールされる前) */
	memset(ls0xx_shadow_fb, 0xff, sizeof(ls0xx_shadow_fb));
	err = ls0xx_update_display(dev, 1, LS0XX_PANEL_HEIGHT, ls0xx_shadow_fb);
	if (err) {
		LOG_ERR("Initial ink fill failed: %d", err);
		return err;
	}
#else
	/* 起動時の残像対策クリア。成功した場合のみ VCOM 反転を開始する */
	err = ls0xx_clear(dev);
	if (err) {
		LOG_ERR("Initial clear failed: %d", err);
		return err;
	}
#endif

	k_work_schedule(&data->vcom_work, K_MSEC(LS0XX_VCOM_PERIOD_MS / 2));
	LOG_INF("Software VCOM toggling every %d ms", LS0XX_VCOM_PERIOD_MS / 2);

#if LS0XX_ROTATION != 0 && CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS > 0
	/* 自己修復リフレッシュの初回スケジュール (ls0xx_heal_work_handler()
	 * 冒頭コメント参照)。ls0xx_clear() 成功後、shadow_fb はまだ全ゼロ
	 * (=クリア直後のパネルと一致) なので初回タイマー発火まで待っても
	 * 不整合は無い。LS0XX_SWVCOM_INIT_FILL_BLACK 有効時は上の黒転ブリッジ
	 * 成功後にしかここへ来ず、shadow=0xff・パネル=黒で同様に一致している */
	k_work_schedule(&data->heal_work, K_MSEC(CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS));
	LOG_INF("Self-healing panel refresh every %d ms", CONFIG_LS0XX_SWVCOM_HEAL_REFRESH_MS);
#endif

	return 0;
}

static const struct ls0xx_config ls0xx_config = {
	/* CS delay 6µs: SCS setup 6µs / hold 2µs (LS011B7DH03) を GPIO CS 制御で確保 */
	.bus = SPI_DT_SPEC_INST_GET(
		0, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		SPI_TRANSFER_LSB | SPI_CS_ACTIVE_HIGH |
		SPI_HOLD_ON_CS | SPI_LOCK_ON, LS0XX_CS_DELAY_US),
#if DT_INST_NODE_HAS_PROP(0, disp_en_gpios)
	.disp_en_gpio = GPIO_DT_SPEC_INST_GET(0, disp_en_gpios),
#endif
};

static struct ls0xx_data ls0xx_data_inst;

/* DEVICE_API(display, name) は Zephyr 4.x のイテラブルセクション方式
 * (STRUCT_SECTION_ITERABLE) に展開される。struct display_driver_api 自体は
 * v3.5 から変更無し (read/get_framebuffer/set_brightness/set_contrast/
 * set_orientation フィールドも残存) だが、上流 v4.1 ベースの ls0xx.c が
 * これらの未実装フィールドを省略するようになったため追従する
 * (display.h の各ラッパーは api->xxx == NULL を -ENOSYS/NULL 扱いする) */
static DEVICE_API(display, ls0xx_driver_api) = {
	.blanking_on = ls0xx_blanking_on,
	.blanking_off = ls0xx_blanking_off,
	.write = ls0xx_write,
	.get_capabilities = ls0xx_get_capabilities,
	.set_pixel_format = ls0xx_set_pixel_format,
};

DEVICE_DT_INST_DEFINE(0, ls0xx_init, NULL, &ls0xx_data_inst, &ls0xx_config,
		      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,
		      &ls0xx_driver_api);
