/*
 *  jit-config.hpp - JIT config utils
 *
 *  Kheperix (C) 2003 Gwenole Beauchesne
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef JIT_CONFIG_H
#define JIT_CONFIG_H

/**
 *	ENABLE_DYNGEN
 *
 *		Define to enable the portable "JIT1" engine based on code
 *		inlining technique as implemented in QEMU.
 **/

#ifndef ENABLE_DYNGEN
#define ENABLE_DYNGEN 0
#endif

/**
 *	DYNGEN_ASM_OPTS
 *
 *		Define to permit host inline asm optimizations. This is
 *		particularly useful to compute emulated condition code
 *		registers.
 **/

#ifndef DYNGEN_ASM_OPTS
#define DYNGEN_ASM_OPTS 0
#endif

/**
 *	Helpers to reach JIT backends headers
 **/

#if defined(__powerpc__) || defined(__ppc__)
#define JIT_TARGET ppc
#endif
#if defined(__i386__)
#define JIT_TARGET x86
#endif
#if defined(__x86_64__)
#define JIT_TARGET amd64
#endif
#if defined(__s390__)
#define JIT_TARGET s390
#endif
#if defined(__alpha__)
#define JIT_TARGET alpha
#endif
#if defined(__ia64__)
#define JIT_TARGET ia64
#endif
#if defined(__sparc__)
#define JIT_TARGET sparc
#endif
#if defined(__arm__)
#define JIT_TARGET arm
#endif
#if defined(__mc68000)
#define JIT_TARGET m68k
#endif
#ifndef JIT_TARGET
#error "Unsupport architecture for JIT1"
#endif

#define JIT_PATH_CONCAT(X, Y)			X/Y
#define JIT_MAKE_HEADER(PATH, HEADER)	<JIT_PATH_CONCAT(PATH,HEADER)>
#define JIT_TARGET_INCLUDE(HEADER)		JIT_MAKE_HEADER(JIT_PATH_CONCAT(cpu/jit,JIT_TARGET),HEADER)

#endif /* JIT_CONFIG_H */