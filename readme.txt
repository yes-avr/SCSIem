SCSI direct access device EMulator (scsiem)

STM32F103マイコンを使用したSCSIダイレクトアクセス
デバイスをエミュレータです。
SDカードにあるHDイメージをSCSI_HDDとして使用でき
ます。

FATファイルシステムのSDカードに以下のいずれかの
HDイメージファイルを作成してください
	"SCSI_ID0.HDS",
	"SCSI_ID1.HDS",
	"SCSI_ID2.HDS",
	"SCSI_ID3.HDS",
	"SCSI_ID4.HDS",
	"SCSI_ID5.HDS",
ファイル名に対応したSCSI_IDのダイレクトアクセス
デバイスをエミュレートします。

起動時にSDカードがセットされていないとピッピッ
とブザーが鳴ってSDカードがセットされるまで待ちます。

SDカードが認識できるフォーマットでない場合はエラー
で停止します。(2.5HzでRIP報告)

SDカードに対応するファイルが全く存在しない場合は
エラーで停止します。(0.8HzでRIP報告)

PB2(maple-miniのuser switch)をON(Lレベル)にする
とHDイメージファイルをクローズして停止します。
(0.3HzでRIP報告)

USART1(Tx1:PA9, Rx2:PA10)に動作メッセージなどを
出力します。(115200 8N1設定で出力例は以下の様)
Info: SCSI direct access device EMulator (scsiem) May  1 2020
Info: version 1.1 debug_level=3
Info: attempt to open HD image 'SCSI_ID0.HDS'.
Info: >>> SCSI_ID= 0 is assumed.
Info: open an HD Image File. 'SCSI_ID0.HDS'.
Info: 10485760 byte (10MiB = 20480BLKs @ 512B/BLK)
Info: attempt to open HD image 'SCSI_ID1.HDS'.
Info: attempt to open HD image 'SCSI_ID2.HDS'.
Info: attempt to open HD image 'SCSI_ID3.HDS'.
Info: >>> SCSI_ID= 3 is assumed.
Info: open an HD Image File. 'SCSI_ID3.HDS'.
Info: 10485760 byte (10MiB = 20480BLKs @ 512B/BLK)
Info: attempt to open HD image 'SCSI_ID4.HDS'.
Info: attempt to open HD image 'SCSI_ID5.HDS'.
Info: scsiem will emulate number of drives: 2
[z]  SCSI_ID map: 0b00001001
HD Image file(s) info:
ID=0 FILE='SCSI_ID0.HDS' SIZE=10485760 BLKS=20480 BLKSIZE=512
ID=3 FILE='SCSI_ID3.HDS' SIZE=10485760 BLKS=20480 BLKSIZE=512

ビルドにはPlatformIOとそのststm32プラットフォーム
対応が必要です。ビルド要旨はHowToBuild.txtを参照。

サポートしてるSCSIコマンドは以下のとおり
	0x03 REQUEST_SENSE
	0x08 READ_6
	0x0a WRITE_6
	0x12 INQUIRY
	0x1a MODE_SENSE_6
	0x25 READ_CAPACITY
	0x28 READ_10
	0x2a WRITE_10
	0x5a MODE_SENSE_10

	上記以外の0x2f以下と下の0x55のコマンドは無視
	(正常として終了)
	0x55 MODE_SELECT_10

サポート外コマンドはセンスキー0x05でエラー終了する。
