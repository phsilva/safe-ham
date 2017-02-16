##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

CFLAGS += -I.. -I../../decadriver

OBJS  = ../common/button_boot.o ../common/peripheral.o ../common/usb.o ../common/dw1000.o ../common/util.o
OBJS += ../../decadriver/deca_device.o ../../decadriver/deca_params_init.o

BINARY = safe-anchor-rx

LDSCRIPT = ../stm32f4-1bitsy.ld

include ../libopencm3.target.mk
