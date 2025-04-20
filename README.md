# STM32F412G-DISCO_SDcardMeasurement

## 概要
STM32F412G-DISCOボードを用いて、SDカードの読み書き性能測定やファイル操作、LCD表示などを行う組み込みシステムプロジェクトです。FatFsを利用したSDカード制御、LCDへの情報表示、USBホスト機能などを備えています。

## 主な機能
- SDカードのマウント・アンマウント、ファイルシステムのフォーマット（FAT12/16/32, exFAT）
- SDカードへのテキスト・バイナリデータ書き込み、読み出し
- 書き込み速度・時間の計測（DWTサイクルカウンタ利用）
- LCD（ST7789H2等）への情報表示
- SDカードの挿抜検出（BSP_SD_IsDetected, GPIO割り込み/タイマー割り込み対応）
- USBホスト機能（USBメモリ等のサポート）
- コマンド入力による各種操作（UART経由）

## ディレクトリ構成
- `Core/` : メインアプリケーション、割り込み、LCD制御、イベントハンドラ等
- `Drivers/` : HAL/BSPドライバ、LCD/SDカード/USB等のハードウェア制御
- `FATFS/` : FatFs本体、SDカードI/F、プラットフォーム依存部
- `USB_HOST/` : USBホスト関連
- `Utility/Fonts/` : LCD用フォントデータ
- `Middlewares/` : サードパーティミドルウェア（FatFs, USB Host Library等）

## 主要ファイル
- `Core/Src/main.c` : メインループ、初期化、コマンド受付
- `Core/Src/event_handler.c` : コマンドごとのSD/LCD/USB操作処理
- `Core/Src/lcd.c` : LCD制御
- `FATFS/App/fatfs.c` : FatFs初期化・管理
- `FATFS/Target/sd_diskio.c` : SDカードI/Oドライバ
- `Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.c` : SDカードBSPドライバ

## SDカード制御
- FatFs（`ff.c`/`ff.h`）を利用し、SDカードのマウント・ファイル操作・フォーマットを実装
- SDカードの抜き差しはBSP_SD_IsDetected()で検出し、割り込みやタイマーで監視
- 抜去時は`f_mount(NULL, ...)`でアンマウントし、リソースを解放
- フォーマット時はユーザーにy/n確認を行い、実行後にファイルシステム情報（FAT種別、クラスタサイズ、容量等）を表示

## LCD制御
- LCD（ST7789H2等）をBSP経由で制御
- レイヤー初期化、色設定、画面クリア、文字列表示等をサポート
- 書き込み速度やファイル操作結果をLCDに表示可能

## コマンドインターフェース
- UART経由でコマンド入力を受け付け、`event_handler.c`で処理
- 例: '1'=Hello表示, '3'=SD書き込み, '4'=バイナリ書き込み, '9'=フォーマット など

## SDカード挿抜検出
- SD検出ピン（CDピン）をGPIO入力として設定
- タイマー割り込みまたはEXTI割り込みでBSP_SD_IsDetected()を呼び、状態変化時にアンマウント等を実施

## USBホスト
- USBメモリ等の接続に対応（USB_HOST/以下）

## ビルド・開発環境
- STM32CubeIDE推奨
- STM32CubeMXで.ioc管理
- FatFs, STM32 HAL/BSP, USB Host Library

## 注意事項
- SDカードのフォーマットは全データ消去を伴うため、y/n確認後に実行
- FatFsの内部関数は直接使用せず、公開APIのみ利用
- SDカード抜去時は必ずアンマウント処理を行うこと

## 拡張・カスタマイズ例
- LCD表示内容のカスタマイズ
- SDカード書き込みサイズや回数の変更
- USBホスト機能の拡張
- SD検出ピンの割り込み/ポーリング方式切替

---

本プロジェクトはSTM32F412G-DISCOのSDカード・LCD・USB機能を活用した組み込みシステムのリファレンス実装です。
