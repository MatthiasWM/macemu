/*
 *  serial_unix.cpp - Serial device driver, Unix specific stuff
 *
 *  Basilisk II (C) 1997-2008 Christian Bauer
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

#include "sysdeps.h"

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <pthread.h>
#include <semaphore.h>
#include <termios.h>
#include <errno.h>
#ifdef __linux__
#include <linux/lp.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#endif
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "cpu_emulation.h"
#include "main.h"
#include "macos_util.h"
#include "prefs.h"
#include "serial.h"
#include "serial_defs.h"

extern "C" {
#include "sshpty.h"
}

#undef DEBUG
#define DEBUG 0
#include "debug.h"

#define MONITOR 0


// IRIX missing or unsupported defines
#ifdef sgi
#ifndef CRTSCTS
#define CRTSCTS CNEW_RTSCTS
#endif
#ifndef B230400
#define B230400 B115200
#endif
#endif


// Missing functions
#ifndef HAVE_CFMAKERAW
static int cfmakeraw(struct termios *termios_p)
{
	termios_p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	termios_p->c_oflag &= ~OPOST;
	termios_p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	termios_p->c_cflag &= ~(CSIZE|PARENB);
	termios_p->c_cflag |= CS8;
	return 0;
}
#endif

// ----------------------------------------------------------------------------
// Driver private variables
class XSERDPort : public SERDPort
{
public:
    XSERDPort(const char *dev);
    virtual ~XSERDPort();
	virtual int16 open(uint16 config);
	virtual int16 prime_in(uint32 pb, uint32 dce);
	virtual int16 prime_out(uint32 pb, uint32 dce);
	virtual int16 control(uint32 pb, uint32 dce, uint16 code);
	virtual int16 status(uint32 pb, uint32 dce, uint16 code);
	virtual int16 close(void);

private:
	bool open_pty(void);
    bool open_tcp_server(int port);
    bool open_tcp_client(const char *addr, int port);
	bool configure(uint16 config);
	void set_handshake(uint32 s, bool with_dtr);
	static void *input_func(void *arg);
	static void *output_func(void *arg);

	const char *device_name;			// Device name
	enum {serial, parallel, pty, midi, tcp_server, tcp_client}
		protocol;						// Type of device
	int fd;								// FD of device
	pid_t pid;							// PID of child process

	bool io_killed;						// Flag: KillIO called, I/O threads must not call deferred tasks
	bool quitting;						// Flag: Quit threads

	pthread_attr_t thread_attr;			// Input/output thread attributes

	bool input_thread_active;			// Flag: Input thread installed
	volatile bool input_thread_cancel;	// Flag: Cancel input thread
	pthread_t input_thread;				// Data input thread
	sem_t input_signal;					// Signal for input thread: execute command
	uint32 input_pb;					// Command parameter for input thread

	bool output_thread_active;			// Flag: Output thread installed
	volatile bool output_thread_cancel;	// Flag: Cancel output thread
	pthread_t output_thread;			// Data output thread
	sem_t output_signal;				// Signal for output thread: execute command
	uint32 output_pb;					// Command parameter for output thread

	struct termios mode;				// Terminal configuration

    void *tcp_thread();                 // thread that handle tcp connections
    static void *tcp_thread_stub(void *arg) { return ((XSERDPort*)arg)->tcp_thread(); }
    void tcp_accept_client();
    void tcp_disconnect_client();
    bool tcp_launch_thread();
    void tcp_kill_thread(bool for_good=false);

    int tcp_socket;                     // tcp server socket for listening
    int command_pipe[2];                // a pipe that communicates between app and tcp thread
    bool tcp_thread_active;             // TCP server thread installed
    pthread_t tcp_thread_handler;       // TCP server thread
    pthread_mutex_t fd_mutex;           // protect fd in threads
};

// ----------------------------------------------------------------------------

/*
 *  Initialization
 */
void SerialInit(void)
{
    // Read serial preferences and create structs for both ports
    the_serd_port[0] = new XSERDPort(PrefsFindString("seriala"));
    the_serd_port[1] = new XSERDPort(PrefsFindString("serialb"));
}


/*
 *  Deinitialization
 */
void SerialExit(void)
{
    delete (XSERDPort *)the_serd_port[0];
    delete (XSERDPort *)the_serd_port[1];
}

// ----------------------------------------------------------------------------

/*
 * Constructor.
 */
XSERDPort::XSERDPort(const char *dev)
:   device_name(0L)
,   protocol(serial)
,   fd(-1)
,   pid(0)
,   io_killed(false)
,   quitting(false)
,   input_thread_active(false)
,   input_thread_cancel(false)
,   input_thread(0)
,   input_signal(0)
,   input_pb(0)
,   output_thread_active(false)
,   output_thread_cancel(false)
,   output_thread(0)
,   output_signal(0)
,   output_pb(0)
,   tcp_socket(-1)
,   tcp_thread_active(false)
,   tcp_thread_handler(0)
{
    command_pipe[0] = command_pipe[1] = -1;
    device_name = dev;
    Set_pthread_attr(&thread_attr, 2);
    pthread_mutex_init(&fd_mutex, 0L);
}


/*
 * Destructor.
 */
XSERDPort::~XSERDPort()
{
    if (input_thread_active) {
        input_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
        pthread_cancel(input_thread);
#endif
        pthread_join(input_thread, NULL);
        sem_destroy(&input_signal);
        input_thread_active = false;
    }
    if (output_thread_active) {
        output_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
        pthread_cancel(output_thread);
#endif
        pthread_join(output_thread, NULL);
        sem_destroy(&output_signal);
        output_thread_active = false;
    }
    tcp_kill_thread(true);
}


/*
 *  Open serial port
 */
int16 XSERDPort::open(uint16 config)
{
	// Don't open NULL name devices
	if (device_name == NULL)
		return openErr;

	// Init variables
	io_killed = false;
	quitting = false;

	// Open port, according to the syntax of the path
	if (device_name[0] == '|') {
		// Open a process via ptys
		if (!open_pty())
			goto open_error;
	}
	else if (!strcmp(device_name, "midi")) {
		// MIDI:  not yet implemented
		return openErr;
	}
    else if (strncmp(device_name, "tcp:", 4)==0) {
        const char *cc = strrchr(device_name, ':');
        int port = atoi(cc+1);
        if (cc==device_name+3) {
            // port only: we are the server
            if (!open_tcp_server(port))
                goto open_error;
        } else {
            // address followed by port: we are the client
            char *address = strdup(device_name);
            address[ cc-device_name ] = 0;
            bool ret = open_tcp_client(address, port);
            if (!ret)
                goto open_error;
        }
    }
    else {
		// Device special file
		fd = ::open(device_name, O_RDWR);
		if (fd < 0)
			goto open_error;

#if defined(__linux__)
		// Parallel port?
		struct stat st;
		if (fstat(fd, &st) == 0)
			if (S_ISCHR(st.st_mode))
				protocol = ((MAJOR(st.st_rdev) == LP_MAJOR) ? parallel : serial);
#elif defined(__FreeBSD__) || defined(__NetBSD__)
		// Parallel port?
		struct stat st;
		if (fstat(fd, &st) == 0)
			if (S_ISCHR(st.st_mode))
				protocol = (((st.st_rdev >> 16) == 16) ? parallel : serial);
#endif
	}

	// Configure port for raw mode
	if (protocol == serial || protocol == pty) {
		if (tcgetattr(fd, &mode) < 0)
			goto open_error;
		cfmakeraw(&mode);
		mode.c_cflag |= HUPCL;
		mode.c_cc[VMIN] = 1;
		mode.c_cc[VTIME] = 0;
		tcsetattr(fd, TCSAFLUSH, &mode);
	}
	configure(config);

	// Start input/output threads
	input_thread_cancel = false;
	output_thread_cancel = false;
	if (sem_init(&input_signal, 0, 0) < 0)
		goto open_error;
	if (sem_init(&output_signal, 0, 0) < 0)
		goto open_error; 
	input_thread_active = (pthread_create(&input_thread, &thread_attr, input_func, this) == 0);
	output_thread_active = (pthread_create(&output_thread, &thread_attr, output_func, this) == 0);
	if (!input_thread_active || !output_thread_active)
		goto open_error;
    if (protocol==tcp_server)
        tcp_launch_thread();
	return noErr;

open_error:
	if (input_thread_active) {
		input_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
		pthread_cancel(input_thread);
#endif
		pthread_join(input_thread, NULL);
		sem_destroy(&input_signal);
		input_thread_active = false;
	}
	if (output_thread_active) {
		output_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
		pthread_cancel(output_thread);
#endif
		pthread_join(output_thread, NULL);
		sem_destroy(&output_signal);
		output_thread_active = false;
	}
    tcp_kill_thread();
    pthread_mutex_lock(&fd_mutex);
	if (fd > 0) {
		::close(fd);
		fd = -1;
	}
    pthread_mutex_unlock(&fd_mutex);
	return openErr;
}


/*
 *  Read data from port
 */
int16 XSERDPort::prime_in(uint32 pb, uint32 dce)
{
	// Send input command to input_thread
	read_done = false;
	read_pending = true;
	input_pb = pb;
	WriteMacInt32(input_dt + serdtDCE, dce);
	sem_post(&input_signal);
	return 1;	// Command in progress
}


/*
 *  Write data to port
 */
int16 XSERDPort::prime_out(uint32 pb, uint32 dce)
{
	// Send output command to output_thread
	write_done = false;
	write_pending = true;
	output_pb = pb;
	WriteMacInt32(output_dt + serdtDCE, dce);
	sem_post(&output_signal);
	return 1;	// Command in progress
}


/*
 *	Control calls
 */
int16 XSERDPort::control(uint32 pb, uint32 dce, uint16 code)
{
    int i;
	switch (code) {
		case 1:			// KillIO
            bug("**** Calling KillIO\n");
			io_killed = true;
			if (protocol == serial)
				tcflush(fd, TCIOFLUSH);
            for (i=0; i<100; i++) { // wait no longer than 1 sec. to avoid a hanging machine!
                if (!read_pending && !write_pending) break;
				usleep(10000);
            }
            read_pending = write_pending = false;
			io_killed = false;
			return noErr;

		case kSERDConfiguration:
			if (configure(ReadMacInt16(pb + csParam)))
				return noErr;
			else
				return paramErr;

		case kSERDInputBuffer:
			return noErr;	// Not supported under Unix

		case kSERDSerHShake:
			set_handshake(pb + csParam, false);
			return noErr;

		case kSERDSetBreak:
			if (protocol == serial)
				tcsendbreak(fd, 0);
			return noErr;

		case kSERDClearBreak:
			return noErr;

		case kSERDBaudRate: {
			if (protocol != serial)
				return noErr;
			uint16 rate = ReadMacInt16(pb + csParam);
			speed_t baud_rate;
			if (rate <= 50) {
				rate = 50; baud_rate = B50;
			} else if (rate <= 75) {
				rate = 75; baud_rate = B75;
			} else if (rate <= 110) {
				rate = 110; baud_rate = B110;
			} else if (rate <= 134) {
				rate = 134; baud_rate = B134;
			} else if (rate <= 150) {
				rate = 150; baud_rate = B150;
			} else if (rate <= 200) {
				rate = 200; baud_rate = B200;
			} else if (rate <= 300) {
				rate = 300; baud_rate = B300;
			} else if (rate <= 600) {
				rate = 600; baud_rate = B600;
			} else if (rate <= 1200) {
				rate = 1200; baud_rate = B1200;
			} else if (rate <= 1800) {
				rate = 1800; baud_rate = B1800;
			} else if (rate <= 2400) {
				rate = 2400; baud_rate = B2400;
			} else if (rate <= 4800) {
				rate = 4800; baud_rate = B4800;
			} else if (rate <= 9600) {
				rate = 9600; baud_rate = B9600;
			} else if (rate <= 19200) {
				rate = 19200; baud_rate = B19200;
			} else if (rate <= 38400) {
				rate = 38400; baud_rate = B38400;
			} else if (rate <= 57600) {
				rate = 57600; baud_rate = B57600;
			} else {
				// Just for safety in case someone wants a rate between 57600 and 65535
				rate = 57600; baud_rate = B57600;
			}
			WriteMacInt16(pb + csParam, rate);
			cfsetispeed(&mode, baud_rate);
			cfsetospeed(&mode, baud_rate);
			tcsetattr(fd, TCSANOW, &mode);
			return noErr;
		}

		case kSERDHandshake:
		case kSERDHandshakeRS232:
			set_handshake(pb + csParam, true);
			return noErr;

		case kSERDMiscOptions:
			if (protocol != serial)
				return noErr;
			if (ReadMacInt8(pb + csParam) & kOptionPreserveDTR)
				mode.c_cflag &= ~HUPCL;
			else
				mode.c_cflag |= HUPCL;
			tcsetattr(fd, TCSANOW, &mode);
			return noErr;

		case kSERDAssertDTR: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_DTR;
			ioctl(fd, TIOCMBIS, &status);
			return noErr;
		}

		case kSERDNegateDTR: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_DTR;
			ioctl(fd, TIOCMBIC, &status);
			return noErr;
		}

		case kSERDSetPEChar:
		case kSERDSetPEAltChar:
			return noErr;	// Not supported under Unix

		case kSERDResetChannel:
			if (protocol == serial)
				tcflush(fd, TCIOFLUSH);
			return noErr;

		case kSERDAssertRTS: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_RTS;
			ioctl(fd, TIOCMBIS, &status);
			return noErr;
		}

		case kSERDNegateRTS: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_RTS;
			ioctl(fd, TIOCMBIC, &status);
			return noErr;
		}

		case kSERD115KBaud:
			if (protocol != serial)
				return noErr;
			cfsetispeed(&mode, B115200);
			cfsetospeed(&mode, B115200);
			tcsetattr(fd, TCSANOW, &mode);
			return noErr;

		case kSERD230KBaud:
		case kSERDSetHighSpeed:
			if (protocol != serial)
				return noErr;
			cfsetispeed(&mode, B230400);
			cfsetospeed(&mode, B230400);
			tcsetattr(fd, TCSANOW, &mode);
			return noErr;

		default:
			printf("WARNING: SerialControl(): unimplemented control code %d\n", code);
			return controlErr;
	}
}


/*
 *	Status calls
 */
int16 XSERDPort::status(uint32 pb, uint32 dce, uint16 code)
{
	switch (code) {
		case kSERDInputCount: {
			int num = 0;
            if (protocol==tcp_server) {
                pthread_mutex_lock(&fd_mutex);
                if (fd!=-1)
                    ioctl(fd, FIONREAD, &num);
                pthread_mutex_unlock(&fd_mutex);
            } else {
                ioctl(fd, FIONREAD, &num);
            }
			WriteMacInt32(pb + csParam, num);
			return noErr;
		}

		case kSERDStatus: {
			uint32 p = pb + csParam;
			WriteMacInt8(p + staCumErrs, cum_errors);
			cum_errors = 0;
			WriteMacInt8(p + staXOffSent, 0);
			WriteMacInt8(p + staXOffHold, 0);
			WriteMacInt8(p + staRdPend, read_pending);
			WriteMacInt8(p + staWrPend, write_pending);
			if (protocol != serial) {
				WriteMacInt8(p + staCtsHold, 0);
				WriteMacInt8(p + staDsrHold, 0);
				WriteMacInt8(p + staModemStatus, dsrEvent | dcdEvent | ctsEvent);
			} else {
				unsigned int status;
				ioctl(fd, TIOCMGET, &status);
				WriteMacInt8(p + staCtsHold, status & TIOCM_CTS ? 0 : 1);
				WriteMacInt8(p + staDsrHold, status & TIOCM_DTR ? 0 : 1);
				WriteMacInt8(p + staModemStatus,
					(status & TIOCM_DSR ? dsrEvent : 0)
					| (status & TIOCM_RI ? riEvent : 0)
					| (status & TIOCM_CD ? dcdEvent : 0)
					| (status & TIOCM_CTS ? ctsEvent : 0));
			}
			return noErr;
		}

		default:
			printf("WARNING: SerialStatus(): unimplemented status code %d\n", code);
			return statusErr;
	}
}


/*
 *	Close serial port
 */
int16 XSERDPort::close()
{
	// Kill threads
	if (input_thread_active) {
		quitting = true;
		sem_post(&input_signal);
		pthread_join(input_thread, NULL);
		input_thread_active = false;
		sem_destroy(&input_signal);
	}
	if (output_thread_active) {
		quitting = true;
		sem_post(&output_signal);
		pthread_join(output_thread, NULL);
		output_thread_active = false;
		sem_destroy(&output_signal);
	}
    tcp_kill_thread();

	// Close port
    pthread_mutex_lock(&fd_mutex);
	if (fd > 0)
		::close(fd);
	fd = -1;
    pthread_mutex_unlock(&fd_mutex);

	// Wait for the subprocess to exit
	if (pid)
		waitpid(pid, NULL, 0);
	pid = 0;

	return noErr;
}


/*
 * Open a process via ptys
 */
bool XSERDPort::open_pty(void)
{
	// Talk to a process via a pty
	char slave[128];
	int slavefd;

	protocol = pty;
	if (!pty_allocate(&fd, &slavefd, slave, sizeof(slave)))
		return false;
		
	fflush(stdout);
	fflush(stderr);
	switch (pid = fork()) {
	case -1:				// error
		return false;
		break;
	case 0:					// child
		::close(fd);

		/* Make the pseudo tty our controlling tty. */
		pty_make_controlling_tty(&slavefd, slave);

		::close(0); dup(slavefd); // Use the slave fd for stdin,
		::close(1); dup(slavefd); // stdout,
		::close(2); dup(slavefd); // and stderr.

		// <should we be more paranoid about closing unused fds?>
		// <should we drop privileges if running setuid?>

		// Let the shell do the dirty work
		execlp("/bin/sh", "/bin/sh", "-c", ++device_name, (char *)NULL);

		// exec failed!
		printf("serial_open:  could not exec %s: %s\n",
			   "/bin/sh", strerror(errno));
		exit(1);
		break;
	default:				// parent
		// Pid was stored above
		break;
	}

	return true;
}


/*
 *  Configure serial port with MacOS config word
 */
bool XSERDPort::configure(uint16 config)
{
	D(bug(" configure %04x\n", config));
	if (protocol != serial)
		return true;

	// Set number of stop bits
	switch (config & 0xc000) {
		case stop10:
			mode.c_cflag &= ~CSTOPB;
			break;
		case stop20:
			mode.c_cflag |= CSTOPB;
			break;
		default:
			return false;
	}

	// Set parity mode
	switch (config & 0x3000) {
		case noParity:
			mode.c_iflag &= ~INPCK;
			mode.c_oflag &= ~PARENB;
			break;
		case oddParity:
			mode.c_iflag |= INPCK;
			mode.c_oflag |= PARENB;
			mode.c_oflag |= PARODD;
			break;
		case evenParity:
			mode.c_iflag |= INPCK;
			mode.c_oflag |= PARENB;
			mode.c_oflag &= ~PARODD;
			break;
		default:
			return false;
	}

	// Set number of data bits
	switch (config & 0x0c00) {
		case data5:
			mode.c_cflag = (mode.c_cflag & ~CSIZE) | CS5;
			break;
		case data6:
			mode.c_cflag = (mode.c_cflag & ~CSIZE) | CS6;
			break;
		case data7:
			mode.c_cflag = (mode.c_cflag & ~CSIZE) | CS7;
			break;
		case data8:
			mode.c_cflag = (mode.c_cflag & ~CSIZE) | CS8;
			break;
	}

	// Set baud rate
	speed_t baud_rate;
	switch (config & 0x03ff) {
		case baud150: baud_rate = B150; break;
		case baud300: baud_rate = B300; break;
		case baud600: baud_rate = B600; break;
		case baud1200: baud_rate = B1200; break;
		case baud1800: baud_rate = B1800; break;
		case baud2400: baud_rate = B2400; break;
		case baud4800: baud_rate = B4800; break;
		case baud9600: baud_rate = B9600; break;
		case baud19200: baud_rate = B19200; break;
		case baud38400: baud_rate = B38400; break;
		case baud57600: baud_rate = B57600; break;
		default:
			return false;
	}
	cfsetispeed(&mode, baud_rate);
	cfsetospeed(&mode, baud_rate);
	tcsetattr(fd, TCSANOW, &mode);
	return true;
}


/*
 *  Set serial handshaking
 */
void XSERDPort::set_handshake(uint32 s, bool with_dtr)
{
	D(bug(" set_handshake %02x %02x %02x %02x %02x %02x %02x %02x\n",
		ReadMacInt8(s + 0), ReadMacInt8(s + 1), ReadMacInt8(s + 2), ReadMacInt8(s + 3),
		ReadMacInt8(s + 4), ReadMacInt8(s + 5), ReadMacInt8(s + 6), ReadMacInt8(s + 7)));
	if (protocol != serial)
		return;

	if (with_dtr) {
		if (ReadMacInt8(s + shkFCTS) || ReadMacInt8(s + shkFDTR))
			mode.c_cflag |= CRTSCTS;
		else
			mode.c_cflag &= ~CRTSCTS;
	} else {
		if (ReadMacInt8(s + shkFCTS))
			mode.c_cflag |= CRTSCTS;
		else
			mode.c_cflag &= ~CRTSCTS;
	}

	D(bug(" %sware flow control\n", mode.c_cflag & CRTSCTS ? "hard" : "soft"));
	tcsetattr(fd, TCSANOW, &mode);
}


/*
 *  Data input thread
 */
void *XSERDPort::input_func(void *arg)
{
    XSERDPort *s = (XSERDPort *)arg;
	while (!s->input_thread_cancel) {
		// Wait for commands
		sem_wait(&s->input_signal);
		if (s->quitting)
			break;

		// Execute command
		void *buf = Mac2HostAddr(ReadMacInt32(s->input_pb + ioBuffer));
		uint32 length = ReadMacInt32(s->input_pb + ioReqCount);
		D(bug("input_func waiting for %ld bytes of data...\n", length));

        int actual = 0;
        int num = 0;
        pthread_mutex_lock(&s->fd_mutex);
        ioctl(s->fd, FIONREAD, &num);
        if (num>0)
            actual = (int)::read(s->fd, buf, length);
        pthread_mutex_unlock(&s->fd_mutex);
        if (num==0)
            usleep(10000); // avoid beiing slammed with read requests
        D(bug(" %ld bytes received\n", actual));

#if MONITOR
		bug("Receiving serial data:\n");
		uint8 *adr = (uint8 *)buf;
		for (int i=0; i<actual; i++) {
			bug("%02x ", adr[i]);
		}
		bug("\n");
#endif

		// KillIO called? Then simply return
		if (s->io_killed) {

			WriteMacInt16(s->input_pb + ioResult, uint16(abortErr));
			WriteMacInt32(s->input_pb + ioActCount, 0);
			s->read_pending = s->read_done = false;

		} else {

			// Set error code
			if (actual >= 0) {
				WriteMacInt32(s->input_pb + ioActCount, actual);
				WriteMacInt32(s->input_dt + serdtResult, noErr);
			} else {
				WriteMacInt32(s->input_pb + ioActCount, 0);
				WriteMacInt32(s->input_dt + serdtResult, uint16(readErr));
			}

			// Trigger serial interrupt
			D(bug(" triggering serial interrupt\n"));
			s->read_done = true;
			SetInterruptFlag(INTFLAG_SERIAL);
			TriggerInterrupt();
		}
	}
	return NULL;
}


/*
 *  Data output thread
 */
void *XSERDPort::output_func(void *arg)
{
	XSERDPort *s = (XSERDPort *)arg;
	while (!s->output_thread_cancel) {

		// Wait for commands
		sem_wait(&s->output_signal);
		if (s->quitting)
			break;

		// Execute command
		void *buf = Mac2HostAddr(ReadMacInt32(s->output_pb + ioBuffer));
		uint32 length = ReadMacInt32(s->output_pb + ioReqCount);
		D(bug("output_func transmitting %ld bytes of data...\n", length));

#if MONITOR
		bug("Sending serial data:\n");
		uint8 *adr = (uint8 *)buf;
		for (int i=0; i<length; i++) {
			bug("%02x ", adr[i]);
		}
		bug("\n");
#endif

        pthread_mutex_lock(&s->fd_mutex);
		int32 actual = s->fd==-1 ? length : (int)write(s->fd, buf, length);
        pthread_mutex_unlock(&s->fd_mutex);
		D(bug(" %ld bytes transmitted\n", actual));

		// KillIO called? Then simply return
		if (s->io_killed) {

			WriteMacInt16(s->output_pb + ioResult, uint16(abortErr));
			WriteMacInt32(s->output_pb + ioActCount, 0);
			s->write_pending = s->write_done = false;

		} else {

			// Set error code
			if (actual >= 0) {
				WriteMacInt32(s->output_pb + ioActCount, actual);
				WriteMacInt32(s->output_dt + serdtResult, noErr);
			} else {
				WriteMacInt32(s->output_pb + ioActCount, 0);
				WriteMacInt32(s->output_dt + serdtResult, uint16(writErr));
			}
	
			// Trigger serial interrupt
			D(bug(" triggering serial interrupt\n"));
			s->write_done = true;
			SetInterruptFlag(INTFLAG_SERIAL);
			TriggerInterrupt();
		}
	}
	return NULL;
}

// ---- tcp/ip ----------------------------------------------------------------

/*
 * Open a process as a tcp server
 * Default port is 3679
 */
bool XSERDPort::open_tcp_server(int port)
{
    struct sockaddr_in serv_addr;

    protocol = tcp_server;

    // create a socket that will listen to incomming tcp connections
    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket < 0) {
        bug("XSERDPort::open_tcp_server: can't create socket: %s\n", strerror(errno));
        return false;
    }

    // make sure that a previously opened socket is killed and does not get stuck in TIMEOUT
    int option = 1;
    setsockopt(tcp_socket, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    // bind the socket to a port
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);
    if (bind(tcp_socket, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        bug("XSERDPort::open_tcp_server: can't bind socket %d to port %d: %s\n", tcp_socket, port, strerror(errno));
        return false;
    }

    // now listen to that port for one incomming connection (but no more than one)
    if (::listen(tcp_socket, 1)==-1) {
        bug("XSERDPort::open_tcp_server: can't listen on socket %d: %s\n", tcp_socket, strerror(errno));
        return false;
    }
    return true;
}


/*
 * Manage incoming connections on our TCP port.
 */
void *XSERDPort::tcp_thread()
{
//    pthread_mutex_lock(&fd_mutex);
//    pthread_mutex_unlock(&fd_mutex);

    fd_set readFDs;
    struct sockaddr_in clientname;

    // Loop forever
    for (;;)
    {
        // Listen to all availabel sockets
        FD_ZERO(&readFDs);
        FD_SET(tcp_socket, &readFDs);
        FD_SET(command_pipe[0], &readFDs);
        pthread_mutex_lock(&fd_mutex);
        if (fd!=-1) FD_SET(fd, &readFDs);
        pthread_mutex_unlock(&fd_mutex);

        // Wait for something to happen
        int nEvents = select(FD_SETSIZE, &readFDs, NULL, NULL, NULL);
        if (nEvents==-1) {
            bug("XSERDPort::tcp_func: error waiting for TCP client: %s\n", strerror(errno));
        }

        // Handle events form an existing client
        pthread_mutex_lock(&fd_mutex);
        bool handleClient = (fd!=-1 && FD_ISSET(fd, &readFDs));
        pthread_mutex_unlock(&fd_mutex);
        if (handleClient) {
            char buf[1];
            pthread_mutex_lock(&fd_mutex);
            fprintf(stderr, "-->");
            size_t ret  = ::recv(fd, buf, 1, MSG_PEEK);
            fprintf(stderr, "<--\n");
            pthread_mutex_unlock(&fd_mutex);
            if (ret==-1) { // error
                if (errno==EAGAIN || errno==EWOULDBLOCK) {
                    // that's ok
                } else {
                    bug("XSERDPort::tcp_func: client read error: %s\n", strerror(errno));
                    tcp_disconnect_client();
                }
            }
            if (ret==0) { // disconnect
                bug("XSERDPort::tcp_func: client disconnected\n");
                tcp_disconnect_client();
            }
        }

        // Handle incoming client connection requests
        if (FD_ISSET(tcp_socket, &readFDs)) {
            // a client wants to connect to us
            bug("--- sel: request connect\n");
            tcp_disconnect_client();
            socklen_t size = sizeof(clientname);
            int tmp_fd = ::accept(tcp_socket, (struct sockaddr *) &clientname, &size);
            if (tmp_fd<0) {
                bug("XSERDPort::tcp_func: error accepting TCP client: %s\n", strerror(errno));
            }
            fcntl(tmp_fd, F_SETFL, O_NONBLOCK);
            bug("tcp_func: connect from host %s, port %hd.\n",
                  inet_ntoa (clientname.sin_addr),
                  ntohs (clientname.sin_port));
            pthread_mutex_lock(&fd_mutex);
            fd = tmp_fd;
            pthread_mutex_unlock(&fd_mutex);
        }

        // Receive a command from another thread
        if (FD_ISSET(command_pipe[0], &readFDs)) {
            uint8_t cmd = '#';
            ::read(command_pipe[0], &cmd, 1);
            switch (cmd) {
                case 'q':
                    ::close(tcp_socket); // not accepting any more incoming client requests
                    tcp_socket = -1;
                    return NULL;
                default:
                    bug("XSERDPort::tcp_thread: unknown command '%c'\n", cmd);
            }
            break;
        }
    }
    return NULL;
}


bool XSERDPort::open_tcp_client(const char *addr, int port)
{
    return false;
}

void XSERDPort::tcp_accept_client()
{
}

void XSERDPort::tcp_disconnect_client()
{
    pthread_mutex_lock(&fd_mutex);
    if (fd!=-1) {
        ::close(fd);
        fd = -1;
    }
    pthread_mutex_unlock(&fd_mutex);
//    sem_post(&input_signal);
//    sem_post(&output_signal);
}


bool XSERDPort::tcp_launch_thread()
{
    int ret = 0;
    if (command_pipe[0]==-1 || command_pipe[1]==-1) {
        ret = pipe(command_pipe);
        if (ret==-1) {
            bug("XSERDPort::tcp_launch_thread: error creating command pipe: %s\n", strerror(errno));
            return false;
        }
    }
    ret = pthread_create(&tcp_thread_handler, NULL, tcp_thread_stub, this);
    if (ret!=0) {
        bug("XSERDPort::tcp_launch_thread: error creating thread: %d\n", ret);
        return false;
    }
    tcp_thread_active = true;
    return true;
}


void XSERDPort::tcp_kill_thread(bool for_good)
{
    if (tcp_thread_active)
    {
        ::write(command_pipe[1], "q", 1);
        pthread_join(tcp_thread_handler, NULL);
        tcp_thread_active = false;
    }
    if (for_good) {
        if (command_pipe[0]!=-1) ::close(command_pipe[0]);
        if (command_pipe[1]!=-1) ::close(command_pipe[1]);
    }
}

