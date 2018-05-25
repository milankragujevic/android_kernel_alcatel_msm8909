#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

#[Feature]-Add-BEGIN by TCTSZ. port NFC device driver of nq210 for GOPLAY2 qian.zhao@tcl.com ,2016/02/02
obj-$(CONFIG_NFC_NQ)           += nq210.o # nq-nci.o
#[Feature]-Add-END by TCTSZ. port NFC device driver of nq210 for GOPLAY2 qian.zhao@tcl.com ,2016/02/02


ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
