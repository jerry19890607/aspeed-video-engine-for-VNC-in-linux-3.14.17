#
# Makefile for the AST frame buffer device driver
#

include ${SPXINC}/projdef.mk
ifeq ($(CONFIG_SPX_FEATURE_GLOBAL_FRAME_BUFFER_SUPPORT), YES)
obj-$(CONFIG_FB_AST) += astfb.o

astfb-objs := ast_fb.o ast_fb_mode.o
endif
