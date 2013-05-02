/****************************************************************************/
/*																			*/
/*	Module:			jamstub.c												*/
/*																			*/
/*					Copyright (C) Altera Corporation 1997-2000				*/
/*																			*/
/*	Description:	Main source file for stand-alone JAM test utility.		*/
/*																			*/
/*					Supports Altera ByteBlaster hardware download cable		*/
/*					on Windows 95 and Windows NT operating systems.			*/
/*					(A device driver is required for Windows NT.)			*/
/*																			*/
/*					Also supports BitBlaster hardware download cable on		*/
/*					Windows 95, Windows NT, and UNIX platforms.				*/
/*																			*/
/*	Revisions:		1.1	added dynamic memory allocation						*/
/*					1.11 added multi-page memory allocation for file_buffer */
/*                    to permit DOS version to read files larger than 64K   */
/*					1.2 fixed control port initialization for ByteBlaster	*/
/*					2.2 updated usage message, added support for alternate	*/
/*					  cable types, moved porting macros in jamport.h,		*/
/*					  fixed bug in delay calibration code for 16-bit port	*/
/*					2.3 added support for static memory						*/
/*						fixed /W4 warnings									*/
/*																			*/
/****************************************************************************/

#ifndef NO_ALTERA_STDIO
#define NO_ALTERA_STDIO
#endif

#if ( _MSC_VER >= 800 )
#pragma warning(disable:4115)
#pragma warning(disable:4201)
#pragma warning(disable:4214)
#pragma warning(disable:4514)
#endif

#include "jamport.h"


typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
#define TRUE 1
#define FALSE 0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>

#if defined(USE_STATIC_MEMORY)
	#define N_STATIC_MEMORY_KBYTES ((unsigned int) USE_STATIC_MEMORY)
	#define N_STATIC_MEMORY_BYTES (N_STATIC_MEMORY_KBYTES * 1024)
	#define POINTER_ALIGNMENT sizeof(DWORD)
#else /* USE_STATIC_MEMORY */
	#include <malloc.h>
	#define POINTER_ALIGNMENT sizeof(BYTE)
#endif /* USE_STATIC_MEMORY */
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "jamexprt.h"


#define	JTAG_TDO	0
#define	JTAG_TMS	5
#define	JTAG_TCK	4
#define	JTAG_TDI	6
#define ATOM_GPO3       7

/************************************************************************
*
*	Global variables
*/

/* file buffer for JAM input file */
char *file_buffer = NULL;
long file_pointer = 0L;
long file_length = 0L;

/* delay count for one millisecond delay */
long one_ms_delay = 0L;

/* delay count to reduce the maximum TCK frequency */
int tck_delay = 0;

/* serial port interface available on all platforms */
BOOL jtag_hardware_initialized = FALSE;
char *serial_port_name = NULL;
BOOL specified_com_port = FALSE;
int com_port = -1;
void initialize_jtag_hardware(void);
void close_jtag_hardware(void);

#if defined(USE_STATIC_MEMORY)
	unsigned char static_memory_heap[N_STATIC_MEMORY_BYTES] = { 0 };
#endif /* USE_STATIC_MEMORY */

#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
	unsigned int n_bytes_allocated = 0;
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */

#if defined(MEM_TRACKER)
	unsigned int peak_memory_usage = 0;
	unsigned int peak_allocations = 0;
	unsigned int n_allocations = 0;
#if defined(USE_STATIC_MEMORY)
	unsigned int n_bytes_not_recovered = 0;
#endif /* USE_STATIC_MEMORY */
	const DWORD BEGIN_GUARD = 0x01234567;
	const DWORD END_GUARD = 0x76543210;
#endif /* MEM_TRACKER */

/* function prototypes to allow forward reference */
extern void delay_loop(long count);

/*
*	This structure stores information about each available vector signal
*/
struct VECTOR_LIST_STRUCT
{
	char *signal_name;
	int  hardware_bit;
	int  vector_index;
};

/*
*	Vector signals for ByteBlaster:
*
*	tck (dclk)    = register 0, bit 0
*	tms (nconfig) = register 0, bit 1
*	tdi (data)    = register 0, bit 6
*	tdo (condone) = register 1, bit 7 (inverted!)
*	nstatus       = register 1, bit 4 (not inverted)
*/
struct VECTOR_LIST_STRUCT vector_list[] =
{
	/* add a record here for each vector signal */
	{ "**TCK**",   0, -1 },
	{ "**TMS**",   1, -1 },
	{ "**TDI**",   6, -1 },
	{ "**TDO**",   7, -1 },
	{ "TCK",       0, -1 },
	{ "TMS",       1, -1 },
	{ "TDI",       6, -1 },
	{ "TDO",       7, -1 },
	{ "DCLK",      0, -1 },
	{ "NCONFIG",   1, -1 },
	{ "DATA",      6, -1 },
	{ "CONF_DONE", 7, -1 },
	{ "NSTATUS",   4, -1 }
};

#define VECTOR_SIGNAL_COUNT ((int)(sizeof(vector_list)/sizeof(vector_list[0])))

BOOL verbose = FALSE;

/************************************************************************
*
*	Customized interface functions for JAM interpreter I/O:
*
*	jam_getc()
*	jam_seek()
*	jam_jtag_io()
*	jam_message()
*	jam_delay()
*/

int jam_getc(void)
{
	int ch = EOF;

	if (file_pointer < file_length)
	{
		ch = (int) file_buffer[file_pointer++];
	}

	return (ch);
}

int jam_seek(long offset)
{
	int return_code = EOF;

	if ((offset >= 0L) && (offset < file_length))
	{
		file_pointer = offset;
		return_code = 0;
	}

	return (return_code);
}

int jam_jtag_io(int tms, int tdi, int read_tdo)
{
	int data = 0;
	int tdo = 0;
	int result = 0;

	if (!jtag_hardware_initialized)
	{
		initialize_jtag_hardware();
		jtag_hardware_initialized = TRUE;
	}

	data = ((tdi ? (0x01<<JTAG_TDI) : 0) | (tms ? (0x01<<JTAG_TMS) : 0));
	result = SusiIOWriteMultiEx((0x01<<JTAG_TCK)|(0x01<<JTAG_TDI)|(0x01<<JTAG_TMS)|(0x01<<ATOM_GPO3), data);
	if (result == FALSE) {
		printf("SusiIOWriteMulti() failed\n");
		exit(1);
	}

	if (read_tdo)
	{
		result = SusiIOReadMultiEx(0x01<<JTAG_TDO, &tdo);
		if (result == FALSE) {
			printf("SusiIOReadEx() failed\n");
			exit(1);
		}
	}

	result = SusiIOWriteMultiEx((0x01<<JTAG_TCK)|(0x01<<JTAG_TDI)|(0x01<<JTAG_TMS)|(0x01<<ATOM_GPO3), data|(0x01<<JTAG_TCK));
	if (result == FALSE) {
		printf("SusiIOWriteMulti() failed\n");
		exit(1);
	}

/*	tck_delay = 2500000;
	if (tck_delay != 0) delay_loop(tck_delay);
*/
	result = SusiIOWriteMultiEx((0x01<<JTAG_TCK)|(0x01<<JTAG_TDI)|(0x01<<JTAG_TMS)|(0x01<<ATOM_GPO3), data);
	if (result == FALSE) {
		printf("SusiIOWriteMulti() failed\n");
		exit(1);
	}

	return (tdo);
}

void jam_message(char *message_text)
{
	puts(message_text);
	fflush(stdout);
}

void jam_export_integer(char *key, long value)
{
	if (verbose)
	{
		printf("Export: key = \"%s\", value = %ld\n", key, value);
		fflush(stdout);
	}
}

#define HEX_LINE_CHARS 72
#define HEX_LINE_BITS (HEX_LINE_CHARS * 4)

char conv_to_hex(unsigned long value)
{
	char c;

	if (value > 9)
	{
		c = (char) (value + ('A' - 10));
	}
	else
	{
		c = (char) (value + '0');
	}

	return (c);
}

void jam_export_boolean_array(char *key, unsigned char *data, long count)
{
	unsigned long size, line, lines, linebits, value, j, k;
	char string[HEX_LINE_CHARS + 1];
	long i, offset;

	if (verbose)
	{
		if (count > HEX_LINE_BITS)
		{
			printf("Export: key = \"%s\", %ld bits, value = HEX\n", key, count);
			lines = (count + (HEX_LINE_BITS - 1)) / HEX_LINE_BITS;

			for (line = 0; line < lines; ++line)
			{
				if (line < (lines - 1))
				{
					linebits = HEX_LINE_BITS;
					size = HEX_LINE_CHARS;
					offset = count - ((line + 1) * HEX_LINE_BITS);
				}
				else
				{
					linebits = count - ((lines - 1) * HEX_LINE_BITS);
					size = (linebits + 3) / 4;
					offset = 0L;
				}

				string[size] = '\0';
				j = size - 1;
				value = 0;

				for (k = 0; k < linebits; ++k)
				{
					i = k + offset;
					if (data[i >> 3] & (1 << (i & 7))) value |= (1 << (i & 3));
					if ((i & 3) == 3)
					{
						string[j] = conv_to_hex(value);
						value = 0;
						--j;
					}
				}
				if ((k & 3) > 0) string[j] = conv_to_hex(value);

				printf("%s\n", string);
			}

			fflush(stdout);
		}
		else
		{
			size = (count + 3) / 4;
			string[size] = '\0';
			j = size - 1;
			value = 0;

			for (i = 0; i < count; ++i)
			{
				if (data[i >> 3] & (1 << (i & 7))) value |= (1 << (i & 3));
				if ((i & 3) == 3)
				{
					string[j] = conv_to_hex(value);
					value = 0;
					--j;
				}
			}
			if ((i & 3) > 0) string[j] = conv_to_hex(value);

			printf("Export: key = \"%s\", %ld bits, value = HEX %s\n",
				key, count, string);
			fflush(stdout);
		}
	}
}

void jam_delay(long microseconds)
{
	delay_loop(microseconds *
		((one_ms_delay / 1000L) + ((one_ms_delay % 1000L) ? 1 : 0)));
}

int jam_vector_map
(
	int signal_count,
	char **signals
)
{
	int signal, vector, ch_index, diff;
	int matched_count = 0;
	char l, r;

	for (vector = 0; (vector < VECTOR_SIGNAL_COUNT); ++vector)
	{
		vector_list[vector].vector_index = -1;
	}

	for (signal = 0; signal < signal_count; ++signal)
	{
		diff = 1;
		for (vector = 0; (diff != 0) && (vector < VECTOR_SIGNAL_COUNT);
			++vector)
		{
			if (vector_list[vector].vector_index == -1)
			{
				ch_index = 0;
				do
				{
					l = signals[signal][ch_index];
					r = vector_list[vector].signal_name[ch_index];
					diff = (((l >= 'a') && (l <= 'z')) ? (l - ('a' - 'A')) : l)
						- (((r >= 'a') && (r <= 'z')) ? (r - ('a' - 'A')) : r);
					++ch_index;
				}
				while ((diff == 0) && (l != '\0') && (r != '\0'));

				if (diff == 0)
				{
					vector_list[vector].vector_index = signal;
					++matched_count;
				}
			}
		}
	}

	return (matched_count);
}

int jam_vector_io
(
	int signal_count,
	long *dir_vect,
	long *data_vect,
	long *capture_vect
)
{
	int matched_count = 0;

	printf("jam_vector_io\n");
	if (!jtag_hardware_initialized)
	{
		initialize_jtag_hardware();
		jtag_hardware_initialized = TRUE;
	}

	return (matched_count);
}

int jam_set_frequency(long hertz)
{
	if (verbose)
	{
		printf("Frequency: %ld Hz\n", hertz);
		fflush(stdout);
	}

	if (hertz == -1)
	{
		/* no frequency limit */
		tck_delay = 0;
	}
	else if (hertz == 0)
	{
		/* stop the clock */
		tck_delay = -1;
	}
	else
	{
		/* set the clock delay to the period */
		/* corresponding to the selected frequency */
		tck_delay = (one_ms_delay * 1000) / hertz;
	}

	return (0);
}

void *jam_malloc(unsigned int size)
{	unsigned int n_bytes_to_allocate = 
#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
		sizeof(unsigned int) +
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */
#if defined(MEM_TRACKER)
		(2 * sizeof(DWORD)) +
#endif /* MEM_TRACKER */
		(POINTER_ALIGNMENT * ((size + POINTER_ALIGNMENT - 1) / POINTER_ALIGNMENT));

	unsigned char *ptr = 0;


#if defined(MEM_TRACKER)
	if ((n_bytes_allocated + n_bytes_to_allocate) > peak_memory_usage)
	{
		peak_memory_usage = n_bytes_allocated + n_bytes_to_allocate;
	}
	if ((n_allocations + 1) > peak_allocations)
	{
		peak_allocations = n_allocations + 1;
	}
#endif /* MEM_TRACKER */

#if defined(USE_STATIC_MEMORY)
	if ((n_bytes_allocated + n_bytes_to_allocate) <= N_STATIC_MEMORY_BYTES)
	{
		ptr = (&(static_memory_heap[n_bytes_allocated]));
	}
#else /* USE_STATIC_MEMORY */ 
	ptr = (unsigned char *) malloc(n_bytes_to_allocate);
#endif /* USE_STATIC_MEMORY */

#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
	if (ptr != 0)
	{
		unsigned int i = 0;

#if defined(MEM_TRACKER)
		for (i = 0; i < sizeof(DWORD); ++i)
		{
			*ptr = (unsigned char) (BEGIN_GUARD >> (8 * i));
			++ptr;
		}
#endif /* MEM_TRACKER */

		for (i = 0; i < sizeof(unsigned int); ++i)
		{
			*ptr = (unsigned char) (size >> (8 * i));
			++ptr;
		}

#if defined(MEM_TRACKER)
		for (i = 0; i < sizeof(DWORD); ++i)
		{
			*(ptr + size + i) = (unsigned char) (END_GUARD >> (8 * i));
			/* don't increment ptr */
		}

		++n_allocations;
#endif /* MEM_TRACKER */

		n_bytes_allocated += n_bytes_to_allocate;
	}
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */

	return ptr;
}

void jam_free(void *ptr)
{
		if
	(
#if defined(MEM_TRACKER)
		(n_allocations > 0) &&
#endif /* MEM_TRACKER */
		(ptr != 0)
	)
	{
		unsigned char *tmp_ptr = (unsigned char *) ptr;

#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
		unsigned int n_bytes_to_free = 0;
		unsigned int i = 0;
		unsigned int size = 0;
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */
#if defined(MEM_TRACKER)
		DWORD begin_guard = 0;
		DWORD end_guard = 0;


		tmp_ptr -= sizeof(DWORD);
#endif /* MEM_TRACKER */
#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
		tmp_ptr -= sizeof(unsigned int);
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */
		ptr = tmp_ptr;

#if defined(MEM_TRACKER)
		for (i = 0; i < sizeof(DWORD); ++i)
		{
			begin_guard |= (((DWORD)(*tmp_ptr)) << (8 * i));
			++tmp_ptr;
		}
#endif /* MEM_TRACKER */

#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
		for (i = 0; i < sizeof(unsigned int); ++i)
		{
			size |= (((unsigned int)(*tmp_ptr)) << (8 * i));
			++tmp_ptr;
		}
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */

#if defined(MEM_TRACKER)
		tmp_ptr += size;

		for (i = 0; i < sizeof(DWORD); ++i)
		{
			end_guard |= (((DWORD)(*tmp_ptr)) << (8 * i));
			++tmp_ptr;
		}

		if ((begin_guard != BEGIN_GUARD) || (end_guard != END_GUARD))
		{
			fprintf(stderr, "Error: memory corruption detected for allocation #%d... bad %s guard\n",
				n_allocations, (begin_guard != BEGIN_GUARD) ? "begin" : "end");
		}

		--n_allocations;
#endif /* MEM_TRACKER */

#if defined(USE_STATIC_MEMORY) || defined(MEM_TRACKER)
		n_bytes_to_free = 
#if defined(MEM_TRACKER)
		(2 * sizeof(DWORD)) +
#endif /* MEM_TRACKER */
		sizeof(unsigned int) +
		(POINTER_ALIGNMENT * ((size + POINTER_ALIGNMENT - 1) / POINTER_ALIGNMENT));
#endif /* USE_STATIC_MEMORY || MEM_TRACKER */

#if defined(USE_STATIC_MEMORY)
		if ((((unsigned long) ptr - (unsigned long) static_memory_heap) + n_bytes_to_free) == (unsigned long) n_bytes_allocated)
		{
			n_bytes_allocated -= n_bytes_to_free;
		}
#if defined(MEM_TRACKER)
		else
		{
			n_bytes_not_recovered += n_bytes_to_free;
		}
#endif /* MEM_TRACKER */
#else /* USE_STATIC_MEMORY */
#if defined(MEM_TRACKER)
		n_bytes_allocated -= n_bytes_to_free;
#endif /* MEM_TRACKER */
		free(ptr);
#endif /* USE_STATIC_MEMORY */
	}
#if defined(MEM_TRACKER)
	else
	{
		if (ptr != 0)
		{
			fprintf(stderr, "Error: attempt to free unallocated memory\n");
		}
	}
#endif /* MEM_TRACKER */
}


/************************************************************************
*
*	get_tick_count() -- Get system tick count in milliseconds
*
*	for DOS, use BIOS function _bios_timeofday()
*	for WINDOWS use GetTickCount() function
*	for UNIX use clock() system function
*/
DWORD get_tick_count(void)
{
	DWORD tick_count = 0L;

	/* assume clock() function returns microseconds */
	tick_count = (DWORD) (clock() / 1000L);

	return (tick_count);
}

#define DELAY_SAMPLES 10
#define DELAY_CHECK_LOOPS 10000

void calibrate_delay(void)
{
	one_ms_delay = 1000L;
}

char *error_text[] =
{
/* JAMC_SUCCESS            0 */ "success",
/* JAMC_OUT_OF_MEMORY      1 */ "out of memory",
/* JAMC_IO_ERROR           2 */ "file access error",
/* JAMC_SYNTAX_ERROR       3 */ "syntax error",
/* JAMC_UNEXPECTED_END     4 */ "unexpected end of file",
/* JAMC_UNDEFINED_SYMBOL   5 */ "undefined symbol",
/* JAMC_REDEFINED_SYMBOL   6 */ "redefined symbol",
/* JAMC_INTEGER_OVERFLOW   7 */ "integer overflow",
/* JAMC_DIVIDE_BY_ZERO     8 */ "divide by zero",
/* JAMC_CRC_ERROR          9 */ "CRC mismatch",
/* JAMC_INTERNAL_ERROR    10 */ "internal error",
/* JAMC_BOUNDS_ERROR      11 */ "bounds error",
/* JAMC_TYPE_MISMATCH     12 */ "type mismatch",
/* JAMC_ASSIGN_TO_CONST   13 */ "assignment to constant",
/* JAMC_NEXT_UNEXPECTED   14 */ "NEXT unexpected",
/* JAMC_POP_UNEXPECTED    15 */ "POP unexpected",
/* JAMC_RETURN_UNEXPECTED 16 */ "RETURN unexpected",
/* JAMC_ILLEGAL_SYMBOL    17 */ "illegal symbol name",
/* JAMC_VECTOR_MAP_FAILED 18 */ "vector signal name not found",
/* JAMC_USER_ABORT        19 */ "execution cancelled",
/* JAMC_STACK_OVERFLOW    20 */ "stack overflow",
/* JAMC_ILLEGAL_OPCODE    21 */ "illegal instruction code",
/* JAMC_PHASE_ERROR       22 */ "phase error",
/* JAMC_SCOPE_ERROR       23 */ "scope error",
/* JAMC_ACTION_NOT_FOUND  24 */ "action not found",
};

#define MAX_ERROR_CODE (int)((sizeof(error_text)/sizeof(error_text[0]))+1)

/************************************************************************/

int main(int argc, char **argv)
{
	BOOL help = FALSE;
	BOOL error = FALSE;
	char *filename = NULL;
	long offset = 0L;
	long error_line = 0L;
	JAM_RETURN_TYPE crc_result = JAMC_SUCCESS;
	JAM_RETURN_TYPE exec_result = JAMC_SUCCESS;
	unsigned short expected_crc = 0;
	unsigned short actual_crc = 0;
	char key[33] = {0};
	char value[257] = {0};
	int exit_status = 0;
	int arg = 0;
	int exit_code = 0;
	int format_version = 0;
	time_t start_time = 0;
	time_t end_time = 0;
	int time_delta = 0;
	char *workspace = NULL;
	char *action = NULL;
	char *init_list[10];
	int init_count = 0;
	FILE *fp = NULL;
	struct stat sbuf;
	long workspace_size = 0;
	char *exit_string = NULL;
	int reset_jtag = 1;

	verbose = FALSE;

	init_list[0] = NULL;

	/* print out the version string and copyright message */
	fprintf(stderr, "Jam STAPL Player Version 2.5 (20040526)\nCopyright (C) 1997-2004 Altera Corporation\n\n");

	for (arg = 1; arg < argc; arg++)
	{
		if (argv[arg][0] == '-')
		{
			switch(toupper(argv[arg][1]))
			{
			case 'A':				/* set action name */
				action = &argv[arg][2];
				if (action[0] == '"') ++action;
				break;

			case 'D':				/* initialization list */
				if (argv[arg][2] == '"')
				{
					init_list[init_count] = &argv[arg][3];
				}
				else
				{
					init_list[init_count] = &argv[arg][2];
				}
				init_list[++init_count] = NULL;
				break;

			case 'R':		/* don't reset the JTAG chain after use */
				reset_jtag = 0;
				break;

			case 'S':				/* set serial port address */
				serial_port_name = &argv[arg][2];
				specified_com_port = TRUE;
				break;

			case 'M':				/* set memory size */
				if (sscanf(&argv[arg][2], "%ld", &workspace_size) != 1)
					error = TRUE;
				if (workspace_size == 0) error = TRUE;
				break;

			case 'H':				/* help */
				help = TRUE;
				break;

			case 'V':				/* verbose */
				verbose = TRUE;
				break;

			default:
				error = TRUE;
				break;
			}
		}
		else
		{
			/* it's a filename */
			if (filename == NULL)
			{
				filename = argv[arg];
			}
			else
			{
				/* error -- we already found a filename */
				error = TRUE;
			}
		}

		if (error)
		{
			fprintf(stderr, "Illegal argument: \"%s\"\n", argv[arg]);
			help = TRUE;
			error = FALSE;
		}
	}

	if (help || (filename == NULL))
	{
		fprintf(stderr, "Usage:  jam [options] <filename>\n");
		fprintf(stderr, "\nAvailable options:\n");
		fprintf(stderr, "    -h          : show help message\n");
		fprintf(stderr, "    -v          : show verbose messages\n");
		fprintf(stderr, "    -a<action>  : specify action name (Jam STAPL)\n");
		fprintf(stderr, "    -d<var=val> : initialize variable to specified value (Jam 1.1)\n");
		fprintf(stderr, "    -d<proc=1>  : enable optional procedure (Jam STAPL)\n");
		fprintf(stderr, "    -d<proc=0>  : disable recommended procedure (Jam STAPL)\n");
		fprintf(stderr, "    -s<port>    : serial port name (for BitBlaster)\n");
		fprintf(stderr, "    -r          : don't reset JTAG TAP after use\n");
		exit_status = 1;
	}
	else if ((workspace_size > 0) &&
		((workspace = (char *) jam_malloc((size_t) workspace_size)) == NULL))
	{
		fprintf(stderr, "Error: can't allocate memory (%d Kbytes)\n",
			(int) (workspace_size / 1024L));
		exit_status = 1;
	}
	else if (access(filename, 0) != 0)
	{
		fprintf(stderr, "Error: can't access file \"%s\"\n", filename);
		exit_status = 1;
	}
	else
	{
		/* get length of file */
		if (stat(filename, &sbuf) == 0) file_length = sbuf.st_size;

		if ((fp = fopen(filename, "rb")) == NULL)
		{
			fprintf(stderr, "Error: can't open file \"%s\"\n", filename);
			exit_status = 1;
		}
		else
		{
			/*
			*	Read entire file into a buffer
			*/

			file_buffer = (char *) jam_malloc((size_t) file_length);

			if (file_buffer == NULL)
			{
				fprintf(stderr, "Error: can't allocate memory (%d Kbytes)\n",
					(int) (file_length / 1024L));
				exit_status = 1;
			}
			else
			{
				if (fread(file_buffer, 1, (size_t) file_length, fp) !=
					(size_t) file_length)
				{
					fprintf(stderr, "Error reading file \"%s\"\n", filename);
					exit_status = 1;
				}
			}

			fclose(fp);
		}

		if (exit_status == 0)
		{
			/*
			*	Calibrate the delay loop function
			*/
			calibrate_delay();

			/*
			*	Check CRC
			*/
			crc_result = jam_check_crc(
				file_buffer, file_length,
				&expected_crc, &actual_crc);

			if (verbose || (crc_result == JAMC_CRC_ERROR))
			{
				switch (crc_result)
				{
				case JAMC_SUCCESS:
					printf("CRC matched: CRC value = %04X\n", actual_crc);
					break;

				case JAMC_CRC_ERROR:
					printf("CRC mismatch: expected %04X, actual %04X\n",
						expected_crc, actual_crc);
					break;

				case JAMC_UNEXPECTED_END:
					printf("Expected CRC not found, actual CRC value = %04X\n",
						actual_crc);
					break;

				default:
					printf("CRC function returned error code %d\n", crc_result);
					break;
				}
			}

			/*
			*	Dump out NOTE fields
			*/
			if (verbose)
			{
				while (jam_get_note(
					file_buffer, file_length,
					&offset, key, value, 256) == 0)
				{
					printf("NOTE \"%s\" = \"%s\"\n", key, value);
				}
			}

			/*
			*	Execute the JAM program
			*/
			time(&start_time);
			exec_result = jam_execute(
				file_buffer, file_length,
				workspace, workspace_size, action, init_list,
				reset_jtag, &error_line, &exit_code, &format_version);
			time(&end_time);

			if (exec_result == JAMC_SUCCESS)
			{
				if (format_version == 2)
				{
					switch (exit_code)
					{
					case  0: exit_string = "Success"; break;
					case  1: exit_string = "Checking chain failure"; break;
					case  2: exit_string = "Reading IDCODE failure"; break;
					case  3: exit_string = "Reading USERCODE failure"; break;
					case  4: exit_string = "Reading UESCODE failure"; break;
					case  5: exit_string = "Entering ISP failure"; break;
					case  6: exit_string = "Unrecognized device"; break;
					case  7: exit_string = "Device revision is not supported"; break;
					case  8: exit_string = "Erase failure"; break;
					case  9: exit_string = "Device is not blank"; break;
					case 10: exit_string = "Device programming failure"; break;
					case 11: exit_string = "Device verify failure"; break;
					case 12: exit_string = "Read failure"; break;
					case 13: exit_string = "Calculating checksum failure"; break;
					case 14: exit_string = "Setting security bit failure"; break;
					case 15: exit_string = "Querying security bit failure"; break;
					case 16: exit_string = "Exiting ISP failure"; break;
					case 17: exit_string = "Performing system test failure"; break;
					default: exit_string = "Unknown exit code"; break;
					}
				}
				else
				{
					switch (exit_code)
					{
					case 0: exit_string = "Success"; break;
					case 1: exit_string = "Illegal initialization values"; break;
					case 2: exit_string = "Unrecognized device"; break;
					case 3: exit_string = "Device revision is not supported"; break;
					case 4: exit_string = "Device programming failure"; break;
					case 5: exit_string = "Device is not blank"; break;
					case 6: exit_string = "Device verify failure"; break;
					case 7: exit_string = "SRAM configuration failure"; break;
					default: exit_string = "Unknown exit code"; break;
					}
				}

				printf("Exit code = %d... %s\n", exit_code, exit_string);
			}
			else if ((format_version == 2) &&
				(exec_result == JAMC_ACTION_NOT_FOUND))
			{
				if ((action == NULL) || (*action == '\0'))
				{
					printf("Error: no action specified for Jam file.\nProgram terminated.\n");
				}
				else
				{
					printf("Error: action \"%s\" is not supported for this Jam file.\nProgram terminated.\n", action);
				}
			}
			else if (exec_result < MAX_ERROR_CODE)
			{
				printf("Error on line %ld: %s.\nProgram terminated.\n",
					error_line, error_text[exec_result]);
			}
			else
			{
				printf("Unknown error code %d\n", exec_result);
			}

			/*
			*	Print out elapsed time
			*/
			if (verbose)
			{
				time_delta = (int) (end_time - start_time);
				printf("Elapsed time = %02u:%02u:%02u\n",
					time_delta / 3600,			/* hours */
					(time_delta % 3600) / 60,	/* minutes */
					time_delta % 60);			/* seconds */
			}
		}
	}

	if (jtag_hardware_initialized) {
		close_jtag_hardware();
		jtag_hardware_initialized = FALSE;
	}
	if (workspace != NULL) jam_free(workspace);
	if (file_buffer != NULL) jam_free(file_buffer);

	#if defined(MEM_TRACKER)
	if (verbose)
	{
#if defined(USE_STATIC_MEMORY)
		fprintf(stdout, "Memory Usage Info: static memory size = %ud (%dKB)\n", N_STATIC_MEMORY_BYTES, N_STATIC_MEMORY_KBYTES);
#endif /* USE_STATIC_MEMORY */
		fprintf(stdout, "Memory Usage Info: peak memory usage = %ud (%dKB)\n", peak_memory_usage, (peak_memory_usage + 1023) / 1024);
		fprintf(stdout, "Memory Usage Info: peak allocations = %d\n", peak_allocations);
#if defined(USE_STATIC_MEMORY)
		if ((n_bytes_allocated - n_bytes_not_recovered) != 0)
		{
			fprintf(stdout, "Memory Usage Info: bytes still allocated = %d (%dKB)\n", (n_bytes_allocated - n_bytes_not_recovered), ((n_bytes_allocated - n_bytes_not_recovered) + 1023) / 1024);
		}
#else /* USE_STATIC_MEMORY */
		if (n_bytes_allocated != 0)
		{
			fprintf(stdout, "Memory Usage Info: bytes still allocated = %d (%dKB)\n", n_bytes_allocated, (n_bytes_allocated + 1023) / 1024);
		}
#endif /* USE_STATIC_MEMORY */
		if (n_allocations != 0)
		{
			fprintf(stdout, "Memory Usage Info: allocations not freed = %d\n", n_allocations);
		}
	}
#endif /* MEM_TRACKER */


	return (exit_status);
}



void initialize_jtag_hardware()
{
	int result, done, op;

	result = SusiDllInit();
	if (result == FALSE) {
		printf("SusiDllInit() failed\n");
		exit(1);
	}

	result = SusiIOAvailable();
	if (result == 0) {
		printf("SusiIOAvailable() failed\n");
		SusiDllUnInit();
		exit(1);
	}

	result = SusiIOWriteMultiEx((0x01<<JTAG_TCK)|(0x01<<JTAG_TDI)|(0x01<<JTAG_TMS)|(0x01<<ATOM_GPO3), 0);
        if (result == FALSE) {
                printf("SusiIOWriteEx() failed\n");
		exit(1);
        }

}

void close_jtag_hardware()
{
	int result;

	result = SusiIOWriteMultiEx((0x01<<JTAG_TCK)|(0x01<<JTAG_TDI)|(0x01<<JTAG_TMS)|(0x01<<ATOM_GPO3), 0);
        if (result == FALSE) {
                printf("SusiIOWriteEx() failed\n");
		exit(1);
        }
	
	result = SusiDllUnInit();
	if (result == FALSE) {
		printf("SusiDllUnInit() failed\n");
		exit(1);
	}

}




#if !defined (DEBUG)
#pragma optimize ("ceglt", off)
#endif

void delay_loop(long count)
{
	while (count != 0L) count--;
}
