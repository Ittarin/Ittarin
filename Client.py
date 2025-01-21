#!/usr/bin/env python
# -*- coding: utf-8 -*-

from machine import Pin, SPI, PWM
import time
import network
import socket
import math

## 設定レジスタ */
CANSTAT      = 0x0E  # CANステータスレジスタ
CANCTRL      = 0x0F  # CAN制御レジスタ 0000
BFPCTRL      = 0x0C  # バッファピン制御レジスタ
TEC          = 0x1C  # 送信エラーカウンタ
REC          = 0x1D  # 受信エラーカウンタ
CNF3         = 0x28  # 設定レジスタ3
CNF2         = 0x29  # 設定レジスタ2
CNF1         = 0x2A  # 設定レジスタ1
CANINTE      = 0x2B  # 割り込み有効化レジスタ
CANINTF      = 0x2C  # 割り込みフラグレジスタ
EFLG         = 0x2D  # エラーフラグレジスタ
TXRTSCTRL    = 0x0D  # 送信要求ピン制御

## 受信フィルタ */
RXF0SIDH     = 0x00  # 受信フィルタ0 スタンダードID 高位
RXF0SIDL     = 0x01  # 受信フィルタ0 スタンダードID 低位
RXF0EID8     = 0x02  # 受信フィルタ0 拡張ID 高位8ビット
RXF0EID0     = 0x03  # 受信フィルタ0 拡張ID 低位8ビット
RXF1SIDH     = 0x04  # 受信フィルタ1 スタンダードID 高位
RXF1SIDL     = 0x05  # 受信フィルタ1 スタンダードID 低位
RXF1EID8     = 0x06  # 受信フィルタ1 拡張ID 高位8ビット
RXF1EID0     = 0x07  # 受信フィルタ1 拡張ID 低位8ビット
RXF2SIDH     = 0x08  # 受信フィルタ2 スタンダードID 高位
RXF2SIDL     = 0x09  # 受信フィルタ2 スタンダードID 低位
RXF2EID8     = 0x0A  # 受信フィルタ2 拡張ID 高位8ビット
RXF2EID0     = 0x0B  # 受信フィルタ2 拡張ID 低位8ビット
RXF3SIDH     = 0x10  # 受信フィルタ3 スタンダードID 高位
RXF3SIDL     = 0x11  # 受信フィルタ3 スタンダードID 低位
RXF3EID8     = 0x12  # 受信フィルタ3 拡張ID 高位8ビット
RXF3EID0     = 0x13  # 受信フィルタ3 拡張ID 低位8ビット
RXF4SIDH     = 0x14  # 受信フィルタ4 スタンダードID 高位
RXF4SIDL     = 0x15  # 受信フィルタ4 スタンダードID 低位
RXF4EID8     = 0x16  # 受信フィルタ4 拡張ID 高位8ビット
RXF4EID0     = 0x17  # 受信フィルタ4 拡張ID 低位8ビット
RXF5SIDH     = 0x18  # 受信フィルタ5 スタンダードID 高位
RXF5SIDL     = 0x19  # 受信フィルタ5 スタンダードID 低位
RXF5EID8     = 0x1A  # 受信フィルタ5 拡張ID 高位8ビット
RXF5EID0     = 0x1B  # 受信フィルタ5 拡張ID 低位8ビット

## 受信マスク */
RXM0SIDH     = 0x20  # 受信マスク0 スタンダードID 高位
RXM0SIDL     = 0x21  # 受信マスク0 スタンダードID 低位
RXM0EID8     = 0x22  # 受信マスク0 拡張ID 高位8ビット
RXM0EID0     = 0x23  # 受信マスク0 拡張ID 低位8ビット
RXM1SIDH     = 0x24  # 受信マスク1 スタンダードID 高位
RXM1SIDL     = 0x25  # 受信マスク1 スタンダードID 低位
RXM1EID8     = 0x26  # 受信マスク1 拡張ID 高位8ビット
RXM1EID0     = 0x27  # 受信マスク1 拡張ID 低位8ビット

## 送信バッファ0 */
TXB0CTRL     = 0x30  # 送信バッファ0制御
TXB0SIDH     = 0x31  # 送信バッファ0 スタンダードID 高位
TXB0SIDL     = 0x32  # 送信バッファ0 スタンダードID 低位
TXB0EID8     = 0x33  # 送信バッファ0 拡張ID 高位8ビット
TXB0EID0     = 0x34  # 送信バッファ0 拡張ID 低位8ビット
TXB0DLC      = 0x35  # 送信バッファ0 データ長コード
TXB0D0       = 0x36  # 送信バッファ0 データバイト0
TXB0D1       = 0x37  # 送信バッファ0 データバイト1
TXB0D2       = 0x38  # 送信バッファ0 データバイト2
TXB0D3       = 0x39  # 送信バッファ0 データバイト3
TXB0D4       = 0x3A  # 送信バッファ0 データバイト4
TXB0D5       = 0x3B  # 送信バッファ0 データバイト5
TXB0D6       = 0x3C  # 送信バッファ0 データバイト6
TXB0D7       = 0x3D  # 送信バッファ0 データバイト7

## 送信バッファ1 */
TXB1CTRL     = 0x40  # 送信バッファ1制御
TXB1SIDH     = 0x41  # 送信バッファ1 スタンダードID 高位
TXB1SIDL     = 0x42  # 送信バッファ1 スタンダードID 低位
TXB1EID8     = 0x43  # 送信バッファ1 拡張ID 高位8ビット
TXB1EID0     = 0x44  # 送信バッファ1 拡張ID 低位8ビット
TXB1DLC      = 0x45  # 送信バッファ1 データ長コード
TXB1D0       = 0x46  # 送信バッファ1 データバイト0
TXB1D1       = 0x47  # 送信バッファ1 データバイト1
TXB1D2       = 0x48  # 送信バッファ1 データバイト2
TXB1D3       = 0x49  # 送信バッファ1 データバイト3
TXB1D4       = 0x4A  # 送信バッファ1 データバイト4
TXB1D5       = 0x4B  # 送信バッファ1 データバイト5
TXB1D6       = 0x4C  # 送信バッファ1 データバイト6
TXB1D7       = 0x4D  # 送信バッファ1 データバイト7

## 送信バッファ2 */
TXB2CTRL     = 0x50  # 送信バッファ2制御
TXB2SIDH     = 0x51  # 送信バッファ2 スタンダードID 高位
TXB2SIDL     = 0x52  # 送信バッファ2 スタンダードID 低位
TXB2EID8     = 0x53  # 送信バッファ2 拡張ID 高位8ビット
TXB2EID0     = 0x54  # 送信バッファ2 拡張ID 低位8ビット
TXB2DLC      = 0x55  # 送信バッファ2 データ長コード
TXB2D0       = 0x56  # 送信バッファ2 データバイト0
TXB2D1       = 0x57  # 送信バッファ2 データバイト1
TXB2D2       = 0x58  # 送信バッファ2 データバイト2
TXB2D3       = 0x59  # 送信バッファ2 データバイト3
TXB2D4       = 0x5A  # 送信バッファ2 データバイト4
TXB2D5       = 0x5B  # 送信バッファ2 データバイト5
TXB2D6       = 0x5C  # 送信バッファ2 データバイト6
TXB2D7       = 0x5D  # 送信バッファ2 データバイト7

# ## 受信バッファ0 */
RXB0CTRL     = 0x60  # 受信バッファ0制御
RXB0SIDH     = 0x61  # 受信バッファ0 スタンダードID 高位
RXB0SIDL     = 0x62  # 受信バッファ0 スタンダードID 低位
RXB0EID8     = 0x63  # 受信バッファ0 拡張ID 高位8ビット
RXB0EID0     = 0x64  # 受信バッファ0 拡張ID 低位8ビット
RXB0DLC      = 0x65  # 受信バッファ0 データ長コード
RXB0D0       = 0x66  # 受信バッファ0 データバイト0
RXB0D1       = 0x67  # 受信バッファ0 データバイト1
RXB0D2       = 0x68  # 受信バッファ0 データバイト2
RXB0D3       = 0x69  # 受信バッファ0 データバイト3
RXB0D4       = 0x6A  # 受信バッファ0 データバイト4
RXB0D5       = 0x6B  # 受信バッファ0 データバイト5
RXB0D6       = 0x6C  # 受信バッファ0 データバイト6
RXB0D7       = 0x6D  # 受信バッファ0 データバイト7

# ## 受信バッファ1 */
RXB1CTRL     = 0x70  # 受信バッファ1制御
RXB1SIDH     = 0x71  # 受信バッファ1 スタンダードID 高位
RXB1SIDL     = 0x72  # 受信バッファ1 スタンダードID 低位
RXB1EID8     = 0x73  # 受信バッファ1 拡張ID 高位8ビット
RXB1EID0     = 0x74  # 受信バッファ1 拡張ID 低位8ビット
RXB1DLC      = 0x75  # 受信バッファ1 データ長コード
RXB1D0       = 0x76  # 受信バッファ1 データバイト0
RXB1D1       = 0x77  # 受信バッファ1 データバイト1
RXB1D2       = 0x78  # 受信バッファ1 データバイト2
RXB1D3       = 0x79  # 受信バッファ1 データバイト3
RXB1D4       = 0x7A  # 受信バッファ1 データバイト4
RXB1D5       = 0x7B  # 受信バッファ1 データバイト5
RXB1D6       = 0x7C  # 受信バッファ1 データバイト6
RXB1D7       = 0x7D  # 受信バッファ1 データバイト7


# ##******************************************************************
# *               ビットレジスタマスク                              *
# *******************************************************************/

# ## 送信バッファ制御レジスタ (TXBnCTRL) */
TXREQ        = 0x08  # 送信リクエスト
TXP          = 0x03  # 送信優先度

# ## 受信バッファ制御レジスタ (RXBnCTRL) */
RXM          = 0x60  # 受信モード
BUKT         = 0x04  # バッファ利用モード

# ## CAN制御レジスタ (CANCTRL) */
REQOP        = 0xE0  # 動作モード設定
ABAT         = 0x10  # 全送信の中止
OSM          = 0x08  # ワンショットモード
CLKEN        = 0x04  # クロック出力有効化
CLKPRE       = 0x03  # クロックプリスケーラ設定

# ## CANステータスレジスタ (CANSTAT) */
REQOP        = 0xE0  # 現在の動作モード
ICOD         = 0x0E  # 割り込みコード

# ## 割り込み有効化レジスタ (CANINTE) */
RX0IE        = 0x01  # 受信バッファ0割り込み有効化
RX1IE        = 0x02  # 受信バッファ1割り込み有効化
TX0IE        = 0x04  # 送信バッファ0割り込み有効化
TX1IE        = 0x80  # 送信バッファ1割り込み有効化
TX2IE        = 0x10  # 送信バッファ2割り込み有効化
ERRIE        = 0x20  # エラー割り込み有効化
WAKIE        = 0x40  # ウェイクアップ割り込み有効化
MERRE        = 0x80  # メッセージエラー割り込み有効化

# ## 割り込みフラグレジスタ (CANINTF) */
RX0IF        = 0x01  # 受信バッファ0割り込みフラグ
RX1IF        = 0x02  # 受信バッファ1割り込みフラグ
TX0IF        = 0x04  # 送信バッファ0割り込みフラグ
TX1IF        = 0x80  # 送信バッファ1割り込みフラグ
TX2IF        = 0x10  # 送信バッファ2割り込みフラグ
ERRIF        = 0x20  # エラー割り込みフラグ
WAKIF        = 0x40  # ウェイクアップ割り込みフラグ
MERRF        = 0x80  # メッセージエラー割り込みフラグ

# ## バッファピン制御レジスタ (BFPCTRL) */
B1BFS        = 0x20  # バッファ1ピン状態設定
B0BFS        = 0x10  # バッファ0ピン状態設定
B1BFE        = 0x08  # バッファ1ピン出力有効化
B0BFE        = 0x04  # バッファ0ピン出力有効化
B1BFM        = 0x02  # バッファ1モード
B0BFM        = 0x01  # バッファ0モード

# ## 設定レジスタ1 (CNF1) マスク */
SJW          = 0xC0  # 同期ジャンプ幅
BRP          = 0x3F  # ビットレートプリスケーラ

# ## 設定レジスタ2 (CNF2) マスク */
BTLMODE      = 0x80  # 位相セグメント設定
SAM          = 0x40  # サンプルポイント設定
PHSEG1       = 0x38  # 位相セグメント1
PRSEG        = 0x07  # プロパゲーションセグメント

# ## 設定レジスタ3 (CNF3) マスク */
WAKFIL       = 0x40  # ウェイクアップフィルタ
PHSEG2       = 0x07  # 位相セグメント2

# ## 送信要求制御レジスタ (TXRTSCTRL) マスク */
TXB2RTS      = 0x04  # 送信バッファ2リクエスト
TXB1RTS      = 0x02  # 送信バッファ1リクエスト
TXB0RTS      = 0x01  # 送信バッファ0リクエスト


# ##******************************************************************
# *                    ビットタイミング設定                          *
# *******************************************************************/

# ## CNF1 (設定レジスタ1) */
SJW_1TQ      = 0x40  # 同期ジャンプ幅 1TQ
SJW_2TQ      = 0x80  # 同期ジャンプ幅 2TQ
SJW_3TQ      = 0x90  # 同期ジャンプ幅 3TQ
SJW_4TQ      = 0xC0  # 同期ジャンプ幅 4TQ

# ## CNF2 (設定レジスタ2) */
BTLMODE_CNF3 = 0x80  # 位相セグメント2はCNF3で設定
BTLMODE_PH1_IPT  = 0x00  # 位相セグメント2はCNF3で設定しない

SMPL_3X      = 0x40  # サンプルポイントを3回サンプリング
SMPL_1X      = 0x00  # サンプルポイントを1回サンプリング

PHSEG1_8TQ   = 0x38  # 位相セグメント1 8TQ
PHSEG1_7TQ   = 0x30  # 位相セグメント1 7TQ
PHSEG1_6TQ   = 0x28  # 位相セグメント1 6TQ
PHSEG1_5TQ   = 0x20  # 位相セグメント1 5TQ
PHSEG1_4TQ   = 0x18  # 位相セグメント1 4TQ
PHSEG1_3TQ   = 0x10  # 位相セグメント1 3TQ
PHSEG1_2TQ   = 0x08  # 位相セグメント1 2TQ
PHSEG1_1TQ   = 0x00  # 位相セグメント1 1TQ

PRSEG_8TQ    = 0x07  # プロパゲーションセグメント 8TQ
PRSEG_7TQ    = 0x06  # プロパゲーションセグメント 7TQ
PRSEG_6TQ    = 0x05  # プロパゲーションセグメント 6TQ
PRSEG_5TQ    = 0x04  # プロパゲーションセグメント 5TQ
PRSEG_4TQ    = 0x03  # プロパゲーションセグメント 4TQ
PRSEG_3TQ    = 0x02  # プロパゲーションセグメント 3TQ
PRSEG_2TQ    = 0x01  # プロパゲーションセグメント 2TQ
PRSEG_1TQ    = 0x00  # プロパゲーションセグメント 1TQ

# ## CNF3 (設定レジスタ3) */
PHSEG2_8TQ   = 0x07  # 位相セグメント2 8TQ
PHSEG2_7TQ   = 0x06  # 位相セグメント2 7TQ
PHSEG2_6TQ   = 0x05  # 位相セグメント2 6TQ
PHSEG2_5TQ   = 0x04  # 位相セグメント2 5TQ
PHSEG2_4TQ   = 0x03  # 位相セグメント2 4TQ
PHSEG2_3TQ   = 0x02  # 位相セグメント2 3TQ
PHSEG2_2TQ   = 0x01  # 位相セグメント2 2TQ
PHSEG2_1TQ   = 0x00  # 位相セグメント2 1TQ

SOF_ENABLED  = 0x80  # 開始ビット有効化
WAKFIL_ENABLED  = 0x40  # ウェイクアップフィルタ有効化
WAKFIL_DISABLED = 0x00  # ウェイクアップフィルタ無効化


# ##******************************************************************
# *                  制御／設定レジスタ                              *
# *******************************************************************/

# ## CANINTE (割り込み有効化レジスタ) */
RX0IE_ENABLED= 0x01  # 受信バッファ0割り込み有効化
RX0IE_DISABLED  =0x00  # 受信バッファ0割り込み無効化
RX1IE_ENABLED =0x02  # 受信バッファ1割り込み有効化
RX1IE_DISABLED  =0x00  # 受信バッファ1割り込み無効化
G_RXIE_ENABLED  =0x03  # 全受信バッファ割り込み有効化
G_RXIE_DISABLED =0x00  # 全受信バッファ割り込み無効化

TX0IE_ENABLED= 0x04  # 送信バッファ0割り込み有効化
TX0IE_DISABLED  =0x00  # 送信バッファ0割り込み無効化
TX1IE_ENABLED =0x08  # 送信バッファ1割り込み有効化
TX2IE_DISABLED  =0x00  # 送信バッファ2割り込み無効化
TX2IE_ENABLED =0x10  # 送信バッファ2割り込み有効化
TX2IE_DISABLED  =0x00  # 送信バッファ2割り込み無効化
G_TXIE_ENABLED  =0x1C  # 全送信バッファ割り込み有効化
G_TXIE_DISABLED =0x00  # 全送信バッファ割り込み無効化

ERRIE_ENABLED= 0x20  # エラー割り込み有効化
ERRIE_DISABLED  =0x00  # エラー割り込み無効化
WAKIE_ENABLED= 0x40  # ウェイクアップ割り込み有効化
WAKIE_DISABLED  =0x00  # ウェイクアップ割り込み無効化
IVRE_ENABLED = 0x80  # 無効割り込み有効化
IVRE_DISABLED= 0x00  # 無効割り込み無効化

# ## CANINTF (割り込みフラグレジスタ) */
RX0IF_SET    = 0x01  # 受信バッファ0割り込みフラグセット
RX0IF_RESET  = 0x00  # 受信バッファ0割り込みフラグリセット
RX1IF_SET    = 0x02  # 受信バッファ1割り込みフラグセット
RX1IF_RESET  = 0x00  # 受信バッファ1割り込みフラグリセット
TX0IF_SET    = 0x04  # 送信バッファ0割り込みフラグセット
TX0IF_RESET  = 0x00  # 送信バッファ0割り込みフラグリセット
TX1IF_SET    = 0x08  # 送信バッファ1割り込みフラグセット
TX2IF_RESET  = 0x00  # 送信バッファ2割り込みフラグリセット
TX2IF_SET    = 0x10  # 送信バッファ2割り込みフラグセット
TX2IF_RESET  = 0x00  # 送信バッファ2割り込みフラグリセット
ERRIF_SET    = 0x20  # エラー割り込みフラグセット
ERRIF_RESET  = 0x00  # エラー割り込みフラグリセット
WAKIF_SET    = 0x40  # ウェイクアップ割り込みフラグセット
WAKIF_RESET  = 0x00  # ウェイクアップ割り込みフラグリセット
IVRF_SET     = 0x80  # 無効割り込みフラグセット
IVRF_RESET   = 0x00  # 無効割り込みフラグリセット

# ## CANCTRL (制御レジスタ) */
REQOP_CONFIG = 0x80  # 設定モード
REQOP_LISTEN = 0x60  # リスンオンリーモード
REQOP_LOOPBACK  =0x40  # ループバックモード
REQOP_SLEEP  = 0x20  # スリープモード
REQOP_NORMAL = 0x00  # 通常モード

ABORT        = 0x10  # 送信の中止

OSM_ENABLED  = 0x08  # ワンショットモード有効化

CLKOUT_ENABLED = 0x04 # クロック出力有効化
CLKOUT_DISABLED = 0x00 # クロック出力無効化
CLKOUT_PRE_8 = 0x03 # クロック出力プリスケール1/8
CLKOUT_PRE_4 = 0x02 # クロック出力プリスケール1/4
CLKOUT_PRE_2 = 0x01 # クロック出力プリスケール1/2
CLKOUT_PRE_1 = 0x00 # クロック出力プリスケール1/1

# ## CANSTAT (CANステータスレジスタ) */
OPMODE_CONFIG = 0x80  # 設定モード
OPMODE_LISTEN = 0x60  # リスンオンリーモード
OPMODE_LOOPBACK = 0x40  # ループバックモード
OPMODE_SLEEP = 0x20  # スリープモード
OPMODE_NORMAL = 0x00  # 通常モード


# ## RXBnCTRL (受信バッファ制御レジスタ) */
RXM_RCV_ALL = 0x60  # すべてのメッセージを受信
RXM_VALID_EXT = 0x40  # 有効な拡張IDメッセージを受信
RXM_VALID_STD = 0x20  # 有効な標準IDメッセージを受信
RXM_VALID_ALL = 0x00  # 有効なすべてのメッセージを受信

RXRTR_REMOTE = 0x08  # リモートフレーム受信
RXRTR_NO_REMOTE = 0x00  # リモートフレームなし

BUKT_ROLLOVER = 0x04  # 受信バッファロールオーバー有効化
BUKT_NO_ROLLOVER = 0x00  # 受信バッファロールオーバー無効化

FILHIT0_FLTR_1 = 0x01  # フィルタ0が一致
FILHIT0_FLTR_0 = 0x00  # フィルタ0が一致しない

FILHIT1_FLTR_5 = 0x05  # フィルタ5が一致
FILHIT1_FLTR_4 = 0x04  # フィルタ4が一致
FILHIT1_FLTR_3 = 0x03  # フィルタ3が一致
FILHIT1_FLTR_2 = 0x02  # フィルタ2が一致
FILHIT1_FLTR_1 = 0x01  # フィルタ1が一致
FILHIT1_FLTR_0 = 0x00  # フィルタ0が一致


# ## TXBnCTRL (送信バッファ制御レジスタ) */
TXREQ_SET = 0x08  # 送信リクエストセット
TXREQ_CLEAR = 0x00  # 送信リクエストクリア

TXP_HIGHEST = 0x03  # 最優先
TXP_INTER_HIGH = 0x02  # 中高優先
TXP_INTER_LOW = 0x01  # 中低優先
TXP_LOWEST = 0x00  # 最低優先


# ##******************************************************************
# *                  レジスタビットマスク                            *
# *******************************************************************/

DLC_0 = 0x00  # データ長コード 0バイト
DLC_1 = 0x01  # データ長コード 1バイト
DLC_2 = 0x02  # データ長コード 2バイト
DLC_3 = 0x03  # データ長コード 3バイト
DLC_4 = 0x04  # データ長コード 4バイト
DLC_5 = 0x05  # データ長コード 5バイト
DLC_6 = 0x06  # データ長コード 6バイト
DLC_7 = 0x07  # データ長コード 7バイト
DLC_8 = 0x08  # データ長コード 8バイト


# ##******************************************************************
# *                  CAN SPIコマンド                                 *
# *******************************************************************/

CAN_RESET = 0xC0  # CANリセットコマンド
CAN_READ = 0x03  # CANレジスタ読み取りコマンド
CAN_WRITE = 0x02  # CANレジスタ書き込みコマンド
CAN_RTS = 0x80  # リモート送信要求
CAN_RTS_TXB0 = 0x81  # 送信バッファ0のリモート送信要求
CAN_RTS_TXB1 = 0x82  # 送信バッファ1のリモート送信要求
CAN_RTS_TXB2 = 0x84  # 送信バッファ2のリモート送信要求
CAN_RD_STATUS = 0xA0  # ステータス読み取りコマンド
CAN_BIT_MODIFY = 0x05  # ビット修正コマンド
CAN_RX_STATUS = 0xB0  # 受信ステータス読み取りコマンド
CAN_RD_RX_BUFF = 0x90  # 受信バッファ読み取りコマンド
CAN_LOAD_TX = 0x40  # 送信バッファへデータロード


# ##******************************************************************
# *                  その他設定                                      *
# *******************************************************************/

DUMMY_BYTE   = 0x00  # ダミーデータ
TXB0         = 0x31  # 送信バッファ0
TXB1         = 0x41  # 送信バッファ1
TXB2         = 0x51  # 送信バッファ2
RXB0         = 0x61  # 受信バッファ0
RXB1         = 0x71  # 受信バッファ1
EXIDE_SET    = 0x08  # 拡張ID有効化
EXIDE_RESET  = 0x00  # 拡張ID無効化
# #CS   PORTAbits.RA2

# Wi-Fiの設定
SSID = "Buffalo-G-4730"  # Wi-FiのSSID
PASSWORD = "dsbdyj5eivvsh"  # Wi-Fiのパスワード

# Wi-Fi接続
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(SSID, PASSWORD)

print("Connecting to Wi-Fi...")
while not wifi.isconnected():
    time.sleep(1)
print("Wi-Fi connected")
print("IP address:", wifi.ifconfig()[0])

# CAN通信速度設定
CAN_RATE = {
    "5KBPS"   : [0xA7, 0XBF, 0x07],  # 5kbps
    "10KBPS"  : [0x31, 0XA4, 0X04],  # 10kbps
    "20KBPS"  : [0x18, 0XA4, 0x04],  # 20kbps
    "50KBPS"  : [0x09, 0XA4, 0x04],  # 50kbps
    "100KBPS" : [0x04, 0x9E, 0x03],  # 100kbps
    "125KBPS" : [0x03, 0x9E, 0x03],  # 125kbps
    "250KBPS" : [0x01, 0x1E, 0x03],  # 250kbps
    "500KBPS" : [0x00, 0x9E, 0x03],  # 500kbps
    "800KBPS" : [0x00, 0x92, 0x02],  # 800kbps
    "1000KBPS": [0x00, 0x82, 0x02],  # 1000kbps
}

# フラグ定義
gRXFlag = 0
sRXFlag = 0
yRXFlag = 0

# 受信バッファ (8バイト分)
Com_RecBuff = [0, 0, 0, 0, 0, 0, 0, 0]

SPI0_CS1 = 1  # SPIチップセレクト1
SPI0_CS0 = 5  # SPIチップセレクト0

# デバッグモード
debug = True

# MCP2515 CANコントローラ用クラス
class MCP2515():
    def __init__(self):
        # SPI初期化
        self.spi = SPI(0)
        self.spi = SPI(0, 10000000, polarity=0, phase=0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
        self.cs = Pin(SPI0_CS0, Pin.OUT)

    # レジスタ読み取り
    def ReadByte(self, addr):
        self.cs(0)
        self.spi.write(bytearray([CAN_READ]))
        self.spi.write(bytearray([addr]))
        res = self.spi.read(1)
        self.cs(1)
        return int.from_bytes(res, 'big')

    # レジスタ書き込み
    def WriteByte(self, addr):
        self.cs(0)
        self.spi.write(bytearray([addr]))
        self.cs(1)

    # 複数バイト書き込み
    def WriteBytes(self, addr, data):
        self.cs(0)
        self.spi.write(bytearray([CAN_WRITE]))
        self.spi.write(bytearray([addr]))
        self.spi.write(bytearray([data]))
        self.cs(1)

    # リセットコマンド送信
    def Reset(self):
        self.cs(0)
        self.spi.write(bytearray([CAN_RESET]))  # リセットコマンド (0xC0)
        self.cs(1)

    # 初期化処理
    def Init(self, speed="1000KBPS"):
        print("Reset")
        self.Reset()
        time.sleep(0.1)

        # 通信速度設定
        self.WriteBytes(CNF1, CAN_RATE[speed][0])
        self.WriteBytes(CNF2, CAN_RATE[speed][1])
        self.WriteBytes(CNF3, CAN_RATE[speed][2])

        # 送信バッファ設定
        self.WriteBytes(TXB0SIDH, 0xFF)
        self.WriteBytes(TXB0SIDL, 0xE0)
        self.WriteBytes(TXB0DLC, 0x40 | DLC_8)

        # 受信バッファ設定
        self.WriteBytes(RXB0SIDH, 0x00)
        self.WriteBytes(RXB0SIDL, 0x60)
        self.WriteBytes(RXB0CTRL, 0x60)
        self.WriteBytes(RXB0DLC, DLC_8)

        # フィルタ設定
        self.WriteBytes(RXF0SIDH, 0xFF)
        self.WriteBytes(RXF0SIDL, 0xE0)
        self.WriteBytes(RXM0SIDH, 0xFF)
        self.WriteBytes(RXM0SIDL, 0xE0)

        # 割り込み設定
        self.WriteBytes(CANINTF, 0x00)  # 割り込みフラグクリア
        self.WriteBytes(CANINTE, 0x01)  # 受信バッファ0の割り込み有効化
        self.WriteBytes(CANCTRL, REQOP_NORMAL | CLKOUT_ENABLED)  # 通常モード設定

        dummy = self.ReadByte(CANSTAT)
        if OPMODE_NORMAL != (dummy and 0xE0):
            self.WriteBytes(CANCTRL, REQOP_NORMAL | CLKOUT_ENABLED)  # 再設定

    def Send(self, CAN_ID, CAN_TX_Buf, length1):
        try:
            print("送信準備中...")
            tempdata = self.ReadByte(CAN_RD_STATUS)  # ステータス読み取り
            print(f"現在のCANステータス: {hex(tempdata)}")

            # データを送信バッファに書き込み
            self.WriteBytes(TXB0SIDH, (CAN_ID >> 3) & 0xFF)
            self.WriteBytes(TXB0SIDL, (CAN_ID & 0x07) << 5)
            self.WriteBytes(TXB0EID8, 0)
            self.WriteBytes(TXB0EID0, 0)
            self.WriteBytes(TXB0DLC, length1)
            for j in range(0, length1):
                self.WriteBytes(TXB0D0 + j, CAN_TX_Buf[j])

            print("データを送信バッファに書き込み完了")

            # 送信リクエストのステータス確認
            if tempdata & 0x04:
                print("送信バッファがビジー状態、待機中...")
                time.sleep(0.01)
                self.WriteBytes(TXB0CTRL, 0)  # フラグクリア
                while self.ReadByte(CAN_RD_STATUS) & 0x04 != 0:
                    print("送信バッファが解放されるのを待っています...")
                    pass

            print("送信要求開始")
            self.WriteByte(CAN_RTS_TXB0)  # 送信要求
            print("送信完了")
        except Exception as e:
            print("送信中にエラーが発生:", e)

    # データ受信
    def Receive(self, CAN_ID):
        self.WriteBytes(RXB0SIDH, (CAN_ID >> 3) & 0xFF)
        self.WriteBytes(RXB0SIDL, (CAN_ID & 0x07) << 5)
        CAN_RX_Buf = []
        while True:
            if self.ReadByte(CANINTF) & 0x01:  # 受信完了確認
                length = self.ReadByte(RXB0DLC)
                print(length)
                for i in range(0, length):
                    CAN_RX_Buf.append(hex(self.ReadByte(RXB0D0 + i)))
                break
        self.WriteBytes(CANINTF, 0)  # フラグリセット
        self.WriteBytes(CANINTE, 0x01)  # 割り込み再有効化
        self.WriteBytes(RXB0SIDH, 0x00)  # バッファクリア
        self.WriteBytes(RXB0SIDL, 0x60)
        return CAN_RX_Buf

# CAN通信を初期化するクラスのインスタンスを作成
can = MCP2515()

# ギア比の設定
gear_ratio = 3591.0 / 187.0  # ギア比

# PID制御パラメータ
Kp = 0.3  # 比例ゲイン
Ki = 0.0   # 積分ゲイン
Kd = 0.5   # 微分ゲイン

# PID制御用変数
last_error = 0
integral = 0

# 目標角度（rad）
target_angle = 30.0 * math.pi / 180.0  # 30度 → rad

# 誤差が十分小さい場合のしきい値（rad）
ERROR_THRESHOLD = 0.01  # 目標角度との差が ±0.01 rad 以下なら電流を流さない

# 制御ループでのタイムステップ
time_step = 0.1  # 秒

# サーバーのIPアドレスとポート番号
SERVER_IP = '192.168.11.23'
SERVER_PORT = 12345

# MATLABサーバーのIPとポート
MATLAB_SERVER_IP = '192.168.11.21'
MATLAB_SERVER_PORT = 54321

# 初期化関数
def setup():
    print("CANバス初期化中...")
    can.Init("1000KBPS")
    print("CANバス初期化完了")

# 電流値を送信する関数
def send_current_command(current_value):
    try:
        current_value = max(min(current_value, 16384), -16384)
        can_data = [
            (current_value >> 8) & 0xFF,
            current_value & 0xFF,
            0, 0, 0, 0, 0, 0
        ]
        can.Send(0x200, can_data, len(can_data))
        print("送信するCANメッセージ:", [hex(byte) for byte in can_data])
    except Exception as e:
        print("送信エラー:", e)

# サーバーから角度データを受信する関数（radのまま処理）
def receive_from_server(client_socket):
    try:
        data = client_socket.recv(1024)
        if data:
            current_angle_rad = float(data.decode().strip())  # 受信データは既にrad表記
            print(f"サーバーから受信した角度データ: {current_angle_rad:.4f} rad")
            return current_angle_rad
    except Exception as e:
        print(f"サーバーからデータ受信エラー: {e}")
    return None

# MATLABにデータを送信するためのソケットを作成して接続
def connect_to_matlab():
    """MATLABサーバーに接続"""
    matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    matlab_socket.connect((MATLAB_SERVER_IP, MATLAB_SERVER_PORT))
    print("MATLABサーバーに接続しました")
    return matlab_socket

# 最大制御信号（出力電流）を制限
MAX_CONTROL = 600  # 最大5000に制限（以前は16384）

# PID制御関数
def pid_control(target, current):
    global last_error, integral

    # 誤差を計算
    error = target - current
    
    # 誤差がしきい値以内かどうかの判定
    if abs(error) < ERROR_THRESHOLD:
        integral = 0  # 微分項のリセット
        last_error = 0 # 前回の誤差もリセット
        return 0, error

    # 積分項を更新
    integral += error * time_step
    integral = max(min(integral, MAX_INTEGRAL), -MAX_INTEGRAL)

    # 微分項を計算
    derivative = (error - last_error) / time_step

    # 制御信号を計算
    control_signal = Kp * error + Ki * integral + Kd * derivative

    # 前回の誤差を更新
    last_error = error
    
    # 制御信号をハードウェアの限界にクランプ
    control_signal = max(min(control_signal, MAX_CONTROL), -MAX_CONTROL)

    return control_signal, error  # 誤差も返す

# フィードバックデータを受信する関数（deg → rad に変換）
def receive_feedback(matlab_socket):
    try:
        print("データ受信開始")
        received_data = can.Receive(0x201)
        if received_data and len(received_data) == 8:
            # Process received CAN data
            mechanical_angle = (int(received_data[0], 16) << 8) | int(received_data[1], 16)
            angle_in_degrees = (mechanical_angle / 8191.0) * 360.0
            current_angle_rad = angle_in_degrees * math.pi / 180.0  # Fix assignment

            # Print and process additional data
            speed = (int(received_data[2], 16) << 8) | int(received_data[3], 16)
            torque_current = (int(received_data[4], 16) << 8) | int(received_data[5], 16)
            temperature = int(received_data[6], 16)

            print(f"フィードバック: Angle: {current_angle_rad:.4f} rad, Speed: {speed}, "
                  f"Torque Current: {torque_current}, Temperature: {temperature}")

            # Send feedback to MATLAB
            error = target_angle - current_angle_rad
            matlab_message = f"{current_angle_rad:.4f},{error:.4f},{speed},{torque_current},{temperature}\n"
            matlab_socket.sendall(matlab_message.encode())
        else:
            print("無効なフィードバックデータ")
    except Exception as e:
        print(f"受信エラー: {e}")
    
# メインループ（MATLAB送信機能を追加）
def loop(client_socket, matlab_socket):
    print("ループ開始")

    # 最初に固定電流値を送信
    target_current = 600  # 送信する電流値
    print("電流値送信中...")
    send_current_command(target_current)
    print("電流値送信完了")

    # サーバーからの角度データを取得
    current_angle_rad = receive_from_server(client_socket)

    if current_angle_rad is not None:
        print(f"現在の角度: {current_angle_rad:.4f} rad")

        # PID制御を実行して制御信号を計算
        control_signal, error = pid_control(target_angle, current_angle_rad)

        # 誤差が小さい場合は電流を流さない
        if abs(error) < ERROR_THRESHOLD:
            integral = 0
            control_signal = 0
            print("誤差が小さいため電流を流しません")

        control_signal = max(min(control_signal, 16384), -16384)  # 範囲制限
        
        # 誤差が小さい場合（±ERROR_THRESHOLD内）なら電流送信を停止
        if abs(error) < ERROR_THRESHOLD:
            print("目標角に到達したため電流送信を停止")
        else:
            print(f"PID制御信号: {control_signal:.2f}, 誤差: {error:.4f} rad")
            send_current_command(control_signal)

        # MATLABに誤差データを送信
        try:
            matlab_socket.sendall(f"{error:.4f}\n".encode())
            print(f"MATLABに送信した誤差データ: {error:.4f} rad")
        except Exception as e:
            print(f"MATLABへの送信エラー: {e}")

        # PID制御信号を電流値として送信
        print("PID制御による電流値送信中...")
        send_current_command(control_signal)
        print("PID制御による電流値送信完了")

    # フィードバックデータ受信（MATLABに送信）
    print("フィードバック受信中...")
    receive_feedback(matlab_socket)
    print("フィードバック受信完了")

    # 制御ループの遅延
    time.sleep(0.1)

# メイン実行部分
if __name__ == "__main__":
    setup()

    # サーバー（Pico）に接続
    client_socket = socket.socket()
    client_socket.connect((SERVER_IP, SERVER_PORT))
    print("サーバー（Pico）に接続しました")

    # MATLABサーバーに接続
    matlab_socket = connect_to_matlab()

    try:
        while True:
            loop(client_socket, matlab_socket)
    except KeyboardInterrupt:
        print("終了します")
    finally:
        client_socket.close()
        matlab_socket.close()



