SCSI direct access device EMulator (scsiem)

STM32F103�}�C�R�����g�p����SCSI�_�C���N�g�A�N�Z�X
�f�o�C�X���G�~�����[�^�ł��B
SD�J�[�h�ɂ���HD�C���[�W��SCSI_HDD�Ƃ��Ďg�p�ł�
�܂��B

FAT�t�@�C���V�X�e����SD�J�[�h�Ɉȉ��̂����ꂩ��
HD�C���[�W�t�@�C�����쐬���Ă�������
	"SCSI_ID0.HDS",
	"SCSI_ID1.HDS",
	"SCSI_ID2.HDS",
	"SCSI_ID3.HDS",
	"SCSI_ID4.HDS",
	"SCSI_ID5.HDS",
�t�@�C�����ɑΉ�����SCSI_ID�̃_�C���N�g�A�N�Z�X
�f�o�C�X���G�~�����[�g���܂��B

�N������SD�J�[�h���Z�b�g����Ă��Ȃ��ƃs�b�s�b
�ƃu�U�[������SD�J�[�h���Z�b�g�����܂ő҂��܂��B

SD�J�[�h���F���ł���t�H�[�}�b�g�łȂ��ꍇ�̓G���[
�Œ�~���܂��B(2.5Hz��RIP��)

SD�J�[�h�ɑΉ�����t�@�C�����S�����݂��Ȃ��ꍇ��
�G���[�Œ�~���܂��B(0.8Hz��RIP��)

PB2(maple-mini��user switch)��ON(L���x��)�ɂ���
��HD�C���[�W�t�@�C�����N���[�Y���Ē�~���܂��B
(0.3Hz��RIP��)

USART1(Tx1:PA9, Rx2:PA10)�ɓ��상�b�Z�[�W�Ȃǂ�
�o�͂��܂��B(115200 8N1�ݒ�ŏo�͗�͈ȉ��̗l)
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

�r���h�ɂ�PlatformIO�Ƃ���ststm32�v���b�g�t�H�[��
�Ή����K�v�ł��B�r���h�v�|��HowToBuild.txt���Q�ƁB

�T�|�[�g���Ă�SCSI�R�}���h�͈ȉ��̂Ƃ���
	0x03 REQUEST_SENSE
	0x08 READ_6
	0x0a WRITE_6
	0x12 INQUIRY
	0x1a MODE_SENSE_6
	0x25 READ_CAPACITY
	0x28 READ_10
	0x2a WRITE_10
	0x5a MODE_SENSE_10

	��L�ȊO��0x2f�ȉ��Ɖ���0x55�̃R�}���h�͖���
	(����Ƃ��ďI��)
	0x55 MODE_SELECT_10

�T�|�[�g�O�R�}���h�̓Z���X�L�[0x05�ŃG���[�I������B
