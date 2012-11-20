# Copyright (c) 2012 Qualcomm Atheros, Inc.

include $(TOPDIR)/config.mk

LIB	=  $(obj)lib$(SOC).o

SOBJS-y = timer.o
COBJS-y += clock.o
COBJS-y += gpio.o

SRCS	:= $(SOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS-y) $(SOBJS-y))

all:	 $(obj).depend $(LIB)

$(LIB):	$(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

