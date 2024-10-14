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

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

PROVIDE(__app_start__ = ORIGIN(rom));
PROVIDE(__app_size__ = _data_loadaddr + _edata - _data - __app_start__);
PROVIDE(__app_code_size__ = _etext - __app_start__);
PROVIDE(__app_ram_size__ = _ebss - ORIGIN(ram));
