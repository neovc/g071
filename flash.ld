/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 36K
}

__privileged_functions_region_size__ = 16K;
__privileged_data_region_size__ = 512;

__FLASH_segment_start__ = ORIGIN(rom);
__FLASH_segment_end__ = __FLASH_segment_start__ + LENGTH(rom);

__SRAM_segment_start__ = ORIGIN(ram);
__SRAM_segment_end__ = __SRAM_segment_start__ + LENGTH(ram);

__privileged_functions_start__ = __FLASH_segment_start__;
__privileged_functions_end__ = __privileged_functions_start__ + __privileged_functions_region_size__;

__privileged_data_start__ = __SRAM_segment_start__;
__privileged_data_end__ = __privileged_data_start__ + __privileged_data_region_size__;

/* Enforce emmition of the vector table. */
EXTERN (vector_table)

/* Define the entry point of the output file. */
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
	.text : {
		*(.vectors)	/* Vector table */
		. = ALIGN(4);
		*(privileged_functions)
		. = ALIGN(4);
		FILL(0xDEAD);
		/* Ensure that non-privileged code is placed after the region reserved for
		 * privileged kernel code.
		 */
		/* Note that dot (.) actually refers to the byte offset from the start of
		 * the current section (.privileged_functions in this case). As a result,
		 * setting dot (.) to a value sets the size of the section.
		 */
		. = __privileged_functions_region_size__;

		__syscalls_flash_start__ = .;
		*(freertos_system_calls)
		__syscalls_flash_end__ = .;
		. = ALIGN(4);
		*(.text*)	/* Program code */
		. = ALIGN(4);
		*(.rodata*)	/* Read-only data */
		. = ALIGN(4);
	} >rom

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
	} >rom
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >rom
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

	/*
	 * Another section used by C++ stuff, appears when using newlib with
	 * 64bit (long long) printf support
	 */
	.ARM.extab : {
		*(.ARM.extab*)
	} >rom
	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >rom

	. = ALIGN(4);
	_etext = .;

	.data : {
		. = ALIGN(4);
		*(privileged_data)
		. = ALIGN(4);
		FILL(0xDEAD);
		/* Ensure that non-privileged data is placed after the region reserved for
		 * privileged kernel data.
		 */

		/* Note that dot (.) actually refers to the byte offset from the start of
		 * the current section (.privileged_data in this case). As a result, setting
		 * dot (.) to a value sets the size of the section.
		 */
		. = __privileged_data_region_size__;
		*(.data*) 	/* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	} >ram AT >rom

	_data_loadaddr = LOADADDR(.data);
	_data = _edata - SIZEOF(.data);

	.bss : {
		*(EXCLUDE_FILE (*heap_4.*.o) .bss*)	/* Read-write zero initialized data */
		*(COMMON)
		*heap_4.*.o (.bss*)	/* Read-write zero initialized data */
		. = ALIGN(4);
		_ebss = .;
	} >ram

	_bss_loadaddr = LOADADDR(.bss);

	/*
	 * The .eh_frame section appears to be used for C++ exception handling.
	 * You may need to fix this if you're using C++.
	 */
	/DISCARD/ : { *(.eh_frame) }

	. = ALIGN(4);
	end = .;
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
PROVIDE(__app_size__ = _bss_loadaddr - ORIGIN(rom));
PROVIDE(__app_start__ = ORIGIN(rom));
