/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/un.h>
#else
#include <winsock2.h>
#include "getopt/getopt.h"
#endif

#include <pthread.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#define UNIX_PATH_MAX 108
#endif

static SOCKET s[2];
static int wait_for_start = 0;

static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

static pthread_condattr_t cond_attr; // for setting CLOCK_MONOTONIC

struct llist {
	char *data;
	size_t len;
	struct llist *next;
/* #ifndef _WIN32
        struct timespec ts;
 #endif
*/
};

#ifndef _WIN32
/* sample stream header, for embedding in output stream for clients */
typedef struct {
        uint32_t size;      // size of this header plus number of sample bytes before next header
        double ts;          // double timestamp of first sample in stream
} stream_segment_hdr_t;
#endif

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

static rtlsdr_dev_t *dev = NULL;
static uint32_t samp_rate = 2048000;

static int enable_biastee = 0;
static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;

static volatile int do_exit = 0;
static int usb_buffer_size = 0;

static int * tuner_gains;
static char tuner_gains_buf[1024];

/* parameter cache; some rtlsdr_set_XXX methods don't have a corresponding
   rtlsdr_get_XXX, so we cache successful values here.
*/

static
uint32_t p_gain_mode = 0,
        p_if_gain[7],
        p_test_mode = 0,
        p_agc_mode = 0,
        p_rtl_xtal,
        p_tuner_xtal,
        p_tuner_gain_index,
		p_bias_tee,
        p_streaming = 0;

char * tuner_types[] = {
	"unknown",
	"E4000",
	"FC0012",
	"FC0013",
	"FC2580",
	"R820T",
	"R828D"
};

void usage(void)
{
	printf("rtl_tcp, an I/Q spectrum server for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t[-a listen address]\n"
		"\t[-p listen port or unix domain socket path (default: 1234)]\n"
		"\t[-f frequency to tune to [Hz]]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-b number of buffers (default: 15, set by library)]\n"
		"\t[-B libusb buffer size, multiple of 512 (default: 16 * 32 * 512 bytes, set by library)]\n"
		"\t[-n max number of linked list buffers to keep (default: 500)]\n"
		"\t[-d device index (default: 0)]\n"
                "\t[-t test mode: send RTL2832 internal counter, not real samples]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
                "\t[-P ppm_error (default: 0)]\n");
	exit(1);
}

#ifdef _WIN32
int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
#ifdef _MSC_VER
		tmp -= 11644473600000000Ui64;
#else
		tmp -= 11644473600000000ULL;
#endif
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		fflush(stderr);
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal %d caught, exiting!\n", signum);
        fflush(stderr);
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
        struct timespec ts;
	if(!do_exit && ! wait_for_start) {
                char *dest;
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
                uint32_t needlen;
#ifndef _WIN32
                stream_segment_hdr_t *hdr;
                needlen = len + sizeof(stream_segment_hdr_t);
#else
                needlen = len;
#endif
		rpt->data = (char*)malloc(needlen);
                dest = rpt->data;
#ifndef _WIN32
                /* fill in stream_segment_hdr_t and set dest to point after it  */
                hdr = (stream_segment_hdr_t *) rpt->data;
                clock_gettime(CLOCK_REALTIME, &ts);
                /* set start-of-buffer timestamp to current clock minus (# frames) * rate */
                hdr->ts = ts.tv_sec + ts.tv_nsec / 1.0e9 - (len / 2.0) / samp_rate;
                hdr->size = len + sizeof(stream_segment_hdr_t);
		dest += sizeof(stream_segment_hdr_t);
#endif
		memcpy(dest, buf, len);

		rpt->len = needlen;
		rpt->next = NULL;

		pthread_mutex_lock(&ll_mutex);

		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		} else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}

			if(llbuf_num && llbuf_num == num_queued-2){
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;
#if 0
			if (num_queued > global_numq)
				printf("ll+, now %d\n", num_queued);
			else if (num_queued < global_numq)
				printf("ll-, now %d\n", num_queued);
#endif
			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
}

static void *tcp_worker(void *arg)
{
	struct llist *curelem,*prev;
	int bytesleft,bytessent, index;
	struct timeval tv= {1,0};
	struct timespec ts;
	fd_set writefds;
	int r = 0;

	while(1) {
		if(do_exit)
			pthread_exit(0);

		pthread_mutex_lock(&ll_mutex);
		clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_sec += 5; // timeout in 5 seconds
		r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
		if(r == ETIMEDOUT && ! wait_for_start) {
			pthread_mutex_unlock(&ll_mutex);
			fprintf(stderr, "worker cond timeout\n");
                        fflush(stderr);
			do_exit = 1;
			pthread_exit(NULL);
		}

		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while(curelem != 0) {
			bytesleft = curelem->len;
			index = 0;
			bytessent = 0;
			while(bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(s[1], &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(s[1]+1, NULL, &writefds, NULL, &tv);
				if(r > 0 && ! do_exit) {
					bytessent = send(s[1],	&curelem->data[index], bytesleft, 0);
					bytesleft -= bytessent;
					index += bytessent;
				}
				if(r < 0 || bytessent == SOCKET_ERROR || do_exit) {
					fprintf(stderr, "worker socket bye\n");
                                        fflush(stderr);
					do_exit = 1;
					pthread_exit(NULL);
				}
			}
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
}

static int set_gain_by_index(rtlsdr_dev_t *_dev, unsigned int index)
{
	int res = 0;
	int* gains;
	int count = rtlsdr_get_tuner_gains(_dev, NULL);

	if (count > 0 && (unsigned int)count > index) {
		res = rtlsdr_set_tuner_gain(_dev, tuner_gains[index]);
	}

	return res;
}

#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command{
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif
static void *command_worker(void *arg)
{
	int left, received = 0;
	fd_set readfds;
	struct command cmd={0, 0};
	struct timeval tv= {1, 0};
	int r = 0;
	uint32_t tmp;
#define REPLY_BUFF_SIZE 1400
        char rbuf[REPLY_BUFF_SIZE + 1];

	while(1) {
		left=sizeof(cmd);
		while(left >0) {
			FD_ZERO(&readfds);
			FD_SET(s[0], &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s[0]+1, &readfds, NULL, NULL, &tv);
			if(r > 0) {
				received = recv(s[0], (char*)&cmd+(sizeof(cmd)-left), left, 0);
				left -= received;
			}
			if(received == SOCKET_ERROR || do_exit) {
				fprintf(stderr, "comm recv bye\n");
                                fflush(stderr);
				do_exit = 1;
				pthread_exit(NULL);
			}
		}
		switch(cmd.cmd) {
		case 0x01:
			printf("set freq %d\n", ntohl(cmd.param));
			rtlsdr_set_center_freq(dev,ntohl(cmd.param));
			break;
		case 0x02:
			printf("set sample rate %d\n", ntohl(cmd.param));
			rtlsdr_set_sample_rate(dev, ntohl(cmd.param));
                        samp_rate = cmd.param;
			break;
		case 0x03:
			printf("set gain mode %d\n", ntohl(cmd.param));
			rtlsdr_set_tuner_gain_mode(dev, ntohl(cmd.param));
                        p_gain_mode = ntohl(cmd.param);
			break;
		case 0x04:
			printf("set gain %d\n", ntohl(cmd.param));
			rtlsdr_set_tuner_gain(dev, ntohl(cmd.param));
			break;
		case 0x05:
			printf("set freq correction %d\n", ntohl(cmd.param));
			rtlsdr_set_freq_correction(dev, ntohl(cmd.param));
			break;
		case 0x06:
			tmp = ntohl(cmd.param);
			printf("set if stage %d gain %d\n", tmp >> 16, (short)(tmp & 0xffff));
			rtlsdr_set_tuner_if_gain(dev, tmp >> 16, (short)(tmp & 0xffff));
                        if (tmp >> 16 < 7)
                                p_if_gain[tmp >> 16] = (tmp & 0xffff);
			break;
		case 0x07:
			printf("set test mode %d\n", ntohl(cmd.param));
			rtlsdr_set_testmode(dev, ntohl(cmd.param));
                        p_test_mode = ntohl(cmd.param);
			break;
		case 0x08:
			printf("set agc mode %d\n", ntohl(cmd.param));
			rtlsdr_set_agc_mode(dev, ntohl(cmd.param));
                        p_agc_mode = ntohl(cmd.param);
			break;
		case 0x09:
			printf("set direct sampling %d\n", ntohl(cmd.param));
			rtlsdr_set_direct_sampling(dev, ntohl(cmd.param));
			break;
		case 0x0a:
			printf("set offset tuning %d\n", ntohl(cmd.param));
			rtlsdr_set_offset_tuning(dev, ntohl(cmd.param));
			break;
		case 0x0b:
			printf("set rtl xtal %d\n", ntohl(cmd.param));
			rtlsdr_set_xtal_freq(dev, ntohl(cmd.param), 0);
                        p_rtl_xtal = ntohl(cmd.param);
			break;
		case 0x0c:
			printf("set tuner xtal %d\n", ntohl(cmd.param));
			rtlsdr_set_xtal_freq(dev, 0, ntohl(cmd.param));
			break;
		case 0x0d:
			printf("set tuner gain by index %d\n", ntohl(cmd.param));
			set_gain_by_index(dev, ntohl(cmd.param));
                        p_tuner_gain_index = ntohl(cmd.param);
			break;
		case 0x0e:
			printf("set bias tee %d\n", ntohl(cmd.param));
			rtlsdr_set_bias_tee(dev, (int)ntohl(cmd.param));
			p_bias_tee = (int)ntohl(cmd.param);
			break;
                case 0x60:
                        if (cmd.param) {
                                fprintf(stderr, "start streaming i/q samples\n");
                                fflush(stderr);
                                wait_for_start = 0;
                                p_streaming = 1;
                        } else {
                                fprintf(stderr, "stop streaming i/q samples\n");
                                fflush(stderr);
                                wait_for_start = 1;
                                p_streaming = 0;
                        }
                        break;
		default:
			printf("unknown command %d param %d\n", cmd.cmd, ntohl(cmd.param));
			break;
		}
                rtlsdr_get_xtal_freq(dev, &p_rtl_xtal, &p_tuner_xtal);

                sprintf(rbuf, "{\
\"frequency\": %d,\
\"rate\": %d,\
\"gain_mode\": %d,\
\"tuner_gain\": %d,\
\"freq_correction\": %d,\
\"if_gain1\": %d,\
\"if_gain2\": %d,\
\"if_gain3\": %d,\
\"if_gain4\": %d,\
\"if_gain5\": %d,\
\"if_gain6\": %d,\
\"test_mode\": %d,\
\"agc_mode\": %d,\
\"direct_sampling\": %d,\
\"offset_tuning\": %d,\
\"rtl_xtal\": %d,\
\"tuner_xtal\": %d,\
\"tuner_type\": \"%s\",\
\"tuner_gain_index\": %d,\
\"tuner_gain_values\": [%s],\
\"bias_tee\": %d,\
\"streaming\": %d\
}\n",

                        rtlsdr_get_center_freq(dev),
                        rtlsdr_get_sample_rate(dev),
                        p_gain_mode,
                        rtlsdr_get_tuner_gain(dev),
                        rtlsdr_get_freq_correction(dev),
                        p_if_gain[1],
                        p_if_gain[2],
                        p_if_gain[3],
                        p_if_gain[4],
                        p_if_gain[5],
                        p_if_gain[6],
                        p_test_mode,
                        p_agc_mode,
                        rtlsdr_get_direct_sampling(dev),
                        rtlsdr_get_offset_tuning(dev),
                        p_rtl_xtal,
                        p_tuner_xtal,
                        tuner_types[rtlsdr_get_tuner_type(dev)],
                        p_tuner_gain_index,
                        tuner_gains_buf,
                        p_streaming);
                send(s[0], rbuf, strlen(rbuf), 0);

		cmd.cmd = 0xff;
	}
}

int main(int argc, char **argv)
{
	int r, opt, i;
	char* addr = "127.0.0.1";
	int port = 1234;
	uint32_t frequency = 100000000;
	struct sockaddr_in local, remote;
	uint32_t buf_num = 0;
	int dev_index = 0;
	int dev_given = 0;
	int gain = 0;
	int ppm_error = 0;
	struct llist *curelem,*prev;
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	SOCKET listensocket = 0;
	socklen_t rlen;
	fd_set readfds;
	u_long blockmode = 1;
	dongle_info_t dongle_info;
#ifdef _WIN32
	WSADATA wsd;
	i = WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
	char sock_path[1 + UNIX_PATH_MAX];
	struct sockaddr_un local_u;
	char *tmp;
	int use_unix_sock = 0;

#endif
	int num_cons;
        uint32_t test_mode = 0;

// #ifndef _WIN32
//         /* redirect stderr to a file if it's not a terminal */
//         if (! isatty(fileno(stderr))) {
//                 /* change the file name below to something useful when debuggin */
//                 freopen("/dev/null", "a", stderr);
//         }
// #endif

	while ((opt = getopt(argc, argv, "a:p:f:g:s:b:B:n:d:P:tT")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'a':
			addr = optarg;
			break;
		case 'p':
#ifndef _WIN32
			port = (int)strtol(optarg, &tmp, 0);
			if (*tmp != '\0') {
				strncpy(sock_path, optarg, UNIX_PATH_MAX);
				use_unix_sock = 1;
			}
#else
			port = atoi(optarg);
#endif
			break;
		case 'b':
			buf_num = atoi(optarg);
			break;
		case 'B':
			usb_buffer_size = atoi(optarg);
			break;
		case 'n':
			llbuf_num = atoi(optarg);
			break;
		case 'P':
			ppm_error = atoi(optarg);
			break;
                case 't':
                        test_mode = 1;
                        p_test_mode = test_mode;
                        break;
		case 'T':
			enable_biastee = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc < optind)
		usage();

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(2);
	}

	rtlsdr_open(&dev, (uint32_t)dev_index);
	if (NULL == dev) {
		if (dev_index <= 0xff) {
			fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		} else {
			fprintf(stderr, "Failed to open rtlsdr device %d:%d.\n", dev_index >> 16, (dev_index >> 8) & 0xff);
		}
                fflush(stderr);
		exit(3);
	}

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGHUP, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* Set the tuner error */
	verbose_ppm_set(dev, ppm_error);

	/* Set the sample rate */
	r = rtlsdr_set_sample_rate(dev, samp_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
                fflush(stderr);
        }

	/* Set the frequency */
	r = rtlsdr_set_center_freq(dev, frequency);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
                fflush(stderr);
	} else {
		fprintf(stderr, "Tuned to %i Hz.\n", frequency);
                fflush(stderr);
        }

	if (0 == gain) {
		/* Enable automatic gain */
		r = rtlsdr_set_tuner_gain_mode(dev, 0);
		if (r < 0) {
			fprintf(stderr, "WARNING: Failed to enable automatic gain.\n");
                        fflush(stderr);
                }
	} else {
		/* Enable manual gain */
		r = rtlsdr_set_tuner_gain_mode(dev, 1);
		if (r < 0) {
			fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
                        fflush(stderr);
                }

		/* Set the tuner gain */
		r = rtlsdr_set_tuner_gain(dev, gain);
		if (r < 0)
			fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
		else
			fprintf(stderr, "Tuner gain set to %f dB.\n", gain/10.0);
                fflush(stderr);
	}

        rtlsdr_set_testmode(dev, test_mode);
	rtlsdr_set_bias_tee(dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");
	p_bias_tee = enable_biastee;

	/* Reset endpoint before we start reading from it (mandatory) */
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");
                fflush(stderr);
        }

	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_mutex_init(&ll_mutex, NULL);
	pthread_mutex_init(&exit_cond_lock, NULL);
        pthread_condattr_init(&cond_attr);
        pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
	pthread_cond_init(&cond, &cond_attr);
	pthread_cond_init(&exit_cond, NULL);

#ifndef _WIN32
	if (use_unix_sock) {
		listensocket = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
		if (listensocket < 0) {
			fprintf(stderr, "Error opening unix domain socket.\n");
                        fflush(stderr);
			exit(4);
		}
	} else {
#endif
		memset(&local,0,sizeof(local));
		local.sin_family = AF_INET;
		local.sin_port = htons(port);
		local.sin_addr.s_addr = inet_addr(addr);

		listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifndef _WIN32
	}
#else
	r = 1;
	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
#endif
#ifndef _WIN32
	if (use_unix_sock) {
		memset( (char *) &local_u, 0, sizeof(local_u));
		local_u.sun_family = AF_UNIX;
		strncpy(local_u.sun_path, sock_path, sizeof(local_u.sun_path) - 1);
		if (bind(listensocket, (struct sockaddr *) &local_u, sizeof(local_u)) < 0) {
			fprintf(stderr, "Error binding to unix domain socket at %s\n", sock_path);
                        fflush(stderr);
			exit(5);
		}
	} else {
#endif
		bind(listensocket,(struct sockaddr *)&local,sizeof(local));
#ifdef _WIN32
		ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
		r = fcntl(listensocket, F_GETFL, 0);
		r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
	}
#endif

	listen(listensocket,1);

#ifndef _WIN32
	if (use_unix_sock) {
		printf("Listening on unix domain socket %s\n", sock_path);
                fflush(stdout);
	} else {
#endif
		printf("listening...\nUse the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
		       "(gr-osmosdr) source\n"
		       "to receive samples in GRC and control "
		       "rtl_tcp parameters (frequency, gain, ...).\n",
		       addr, port);
#ifndef _WIN32
	}
#endif

	while(1) {
		num_cons = 0;
		while(1) {
			FD_ZERO(&readfds);
			FD_SET(listensocket, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket+1, &readfds, NULL, NULL, &tv);
			if(do_exit) {
				goto out;
			} else if(r > 0) {
				rlen = sizeof(remote);
				s[num_cons] = accept(listensocket,(struct sockaddr *)&remote, &rlen);
				setsockopt(s[num_cons], SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
				if (num_cons == 0) {
					memset(&dongle_info, 0, sizeof(dongle_info));
					memcpy(&dongle_info.magic, "RTL0", 4);
					r = rtlsdr_get_tuner_type(dev);
					if (r >= 0)
						dongle_info.tuner_type = htonl(r);

					r = rtlsdr_get_tuner_gains(dev, NULL);
					if (r >= 0) {
                                                int j, k;
                                                char *tgb = & tuner_gains_buf[0];
						dongle_info.tuner_gain_count = htonl(r);
                                                tuner_gains = malloc(r * sizeof(int));
                                                rtlsdr_get_tuner_gains(dev, tuner_gains);
                                                /* format this array into tuner_gains_buf */
                                                tuner_gains_buf[0] = '\0';
                                                for (j = 0; j < r; ++j) {
                                                        sprintf(tgb, "%c%.1f%n", (j > 0) ? ',' : ' ', tuner_gains[j] / 10.0, & k);
                                                        tgb += k;
                                                };
                                        }
					r = send(s[0], (const char *)&dongle_info, sizeof(dongle_info), 0);
					if (sizeof(dongle_info) != r)
						printf("failed to send dongle information\n");
				}
				++ num_cons;
#ifndef _WIN32
				if (! use_unix_sock || num_cons == 2)
#endif
					break;
			}
		}

		if (num_cons == 1) {
			s[1] = s[0];
                } else {
                        wait_for_start = 1;
                }

		printf("client accepted!\n");


		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		pthread_attr_destroy(&attr);

		r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, usb_buffer_size);

		pthread_join(tcp_worker_thread, &status);
		pthread_join(command_thread, &status);

		closesocket(s[0]);
		if (num_cons == 2)
			closesocket(s[1]);

		printf("all threads dead..\n");
		curelem = ll_buffers;
		ll_buffers = 0;

		while(curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		do_exit = 1;
		global_numq = 0;
	}

 out:
        fprintf(stderr, "Closing rtlsdr device.\n");
        fflush(stderr);
	rtlsdr_close(dev);
	closesocket(listensocket);
	closesocket(s[0]);
	if (num_cons ==2)
		closesocket(s[1]);
#ifdef _WIN32
	WSACleanup();
#else
	if (use_unix_sock)
		unlink(sock_path);
#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}
