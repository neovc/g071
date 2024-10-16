#!/usr/bin/env python
# encoding: utf-8

import os
import time
from datetime import datetime

from waflib.Build import BuildContext

APPNAME = 'g071'

top = '.'
out = 'build'
cwd = os.getcwd()
now = datetime.now()
bin_date = now.strftime("%y%m%d")
bin_target = "st." + bin_date + ".bin"
appendcrc_bin = cwd + '/' + out + '/append_crc'

libopencm3_revision = os.popen('cd libopencm3 && git log --pretty=format:"%h" --abbrev=8 2> /dev/null | head -1 || echo {0}'.format("1.")).read().strip()
libopencm3_revision_date = os.popen('cd libopencm3 && git log --pretty=format:"%ad" --date=format:"%Y%m%d" 2>/dev/null | head -1 | cut -f 1 -d\' \'').read().strip()

def down_libopencm3():
    if not os.path.exists('libopencm3'):
        print('begin downloading libopencm3 library...\n')
        os.system('git clone git@github.com:libopencm3/libopencm3')

def build_appendcrc():
    if not os.path.exists(appendcrc_bin):
        print('build append_crc ...\n')
        os.system("gcc -O2 -o " + appendcrc_bin + " src/append_crc.c")

def update_libopencm3():
    os.system('cd libopencm3 && git pull && make TARGETS=stm32/g0')

def options(ctx):
    ctx.load('gcc')

    down_libopencm3()
    build_appendcrc()

    ctx.add_option('--arch', action='store', default='cortex-m0plus', help='MCU arch')
    ctx.add_option('--toolchain', action='store', default='arm-none-eabi-', help='Set toolchain prefix')
    ctx.add_option('--update', action='store_true', help='Update libopencm3 source')

def configure(ctx):
    ctx.env.CC = ctx.options.toolchain + "gcc"
    ctx.env.AR = ctx.options.toolchain + "ar"
    ctx.load('gcc')

    # Locate programs
    ctx.find_program('st-flash', var='STFLASH')
    ctx.find_program(appendcrc_bin, var='APPENDCRC')
    ctx.find_program(ctx.options.toolchain + 'size', var='SIZE')
    ctx.find_program(ctx.options.toolchain + 'objcopy', var='OBJCOPY')

    # Generate build arguments
    ctx.env.append_unique('CFLAGS', ['-Wall', '-DSTM32G0', '-fno-common', '-Os', '-mthumb', '-mcpu=cortex-m0plus', '-fno-exceptions', '-ffunction-sections', '-fdata-sections', '-Wempty-body', '-Wtype-limits', '-Wmissing-parameter-type', '-Wuninitialized', '-fno-strict-aliasing', '-Wno-unused-function', '-Wno-stringop-truncation', '-fsingle-precision-constant', '-MD'])

    ctx.env.append_unique('LINKFLAGS', ['--static', '-nostartfiles', '-Wl,--gc-sections', '-mthumb', '-mcpu=cortex-m0plus'])

    ctx.env.append_unique('LDFLAGS', ['--specs=nano.specs', '-Wl,--start-group', '-lc', '-lgcc', '-lnosys', '-Wl,--end-group', '-lm'])

    ctx.env.append_unique('INCLUDES', ['../rtos/include', '../src', '../libopencm3/include'])

    # FreeRTOS
    ctx.env.append_unique('FILES', ['rtos/*.c', 'src/g071.c'])

    ctx.env.append_unique('LINKFLAGS', ['-T' + cwd + '/flash.ld', '-Wl,-Map=flash.map'])

    if ctx.options.update == True or not os.path.exists('libopencm3/lib/libopencm3_stm32g0.a'):
        update_libopencm3()

def build(ctx):
    # Linker script
    hex_target = 'st.hex'

    build_appendcrc()
    ctx.program(
        source=ctx.path.ant_glob(ctx.env.FILES),
        target='st.elf',
        stlib=['opencm3_stm32g0'],
        stlibpath=[cwd + '/libopencm3/lib']
    )
    ctx(rule='${OBJCOPY} -O binary ${SRC} ${TGT}', source='st.elf', target=bin_target, name='objcopy', always=True)
    ctx(rule='${APPENDCRC} -r ${SRC}', source=bin_target, name='append_crc', always=True)
    ctx(name="size", rule='${SIZE} ${SRC}', source='st.elf', always=True)

def flash(ctx):
    ctx(name='flash', rule='${STFLASH} erase && ${STFLASH} write ${SRC} 0x8000000', source=bin_target, always=True)

class Program(BuildContext):
    cmd = 'flash'
    fun = 'flash'
