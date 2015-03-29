
include $(TOPDIR)/config.mk

SOBJS-y = timer.o gpio.o clock.o scm.o

LIB	=  $(obj)lib$(SOC).o

COBJS-y += cmd_bootqca.o
ifeq ($(CONFIG_QCA_APPSBL_DLOAD),y)
COBJS-y	+= cmd_dumpqca_data.o
endif
SRCS	:= $(SOBJS-y:.o=.S) $(COBJS-y:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS-y) $(SOBJS-y))

all:	 $(obj).depend $(LIB)

$(LIB):	$(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
